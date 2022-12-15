/** -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * Copyright (C) 2022 Yu-Tai Lin
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "ns3/core-module.h"
#include "ns3/mac48-address.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac-queue-item.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/mgt-headers.h"
#include "ns3/packet.h"
#include "ns3/mgt-headers.h"

#include "txop.h"
#include "channel-access-manager.h"
#include "centralized-scheduler.h"
#include "scheduler-time-tracker.h"
#include "transmission-selector.h"

#include <vector>
#include <tuple>
#include <queue>

namespace ns3 {

class Txop;
class ChannelAccessManager;

NS_LOG_COMPONENT_DEFINE ("CentralizedScheduler");

NS_OBJECT_ENSURE_REGISTERED (CentralizedScheduler);

CentralizedScheduler::CentralizedScheduler(double in_timeTrackerAlpha, Time in_timeTrackerPeriodLength, TransmissionType in_startTransType)
{
	// initialize time tracker and transmission selector
	m_timeTracker = CreateObject<SchedulerTimeTracker> (in_timeTrackerAlpha, in_timeTrackerPeriodLength);
	m_currTransType = in_startTransType;
	m_grantedTxop = nullptr;
	m_isTransmitting = false;
	m_lock = 0;

	if (in_timeTrackerAlpha != 0) {
		int64_t ftmDurationInt = m_timeTracker->GetFtmTimeLength().GetPicoSeconds();
		Simulator::Schedule(PicoSeconds(ftmDurationInt-1), &CentralizedScheduler::SwitchTransType, this);
	}
}

CentralizedScheduler::~CentralizedScheduler()
{
	m_timeTracker = 0;
	m_grantedTxop = nullptr;
	m_preTransPairAddr.clear();
	m_transPairAddr.clear();
	m_mgtOtherQueue = std::queue<Ptr<Txop>>();
	m_dataTxopQueue = std::queue<Ptr<Txop>>();
	m_ftmTxopQueue = std::queue<Ptr<Txop>>();
	m_broadcastQueue = std::queue<Ptr<Txop>>();
}

void
CentralizedScheduler::DoDispose(void)
{
	NS_LOG_FUNCTION(this);
}

void
CentralizedScheduler::TransmissionStart()
{
	m_isTransmitting = true;
	m_timeTracker->MarkTransmissionStartTime();
}

void
CentralizedScheduler::SwitchTransType()
{
	NS_LOG_DEBUG("Swtich type, current type: " << m_currTransType << ", time: " << Simulator::Now());
	switch (m_currTransType)
	{
		case TransmissionType::DATA:
			m_currTransType = TransmissionType::FTM;
			Simulator::Schedule(m_timeTracker->GetFtmTimeLength(), &CentralizedScheduler::SwitchTransType, this);
			break;

		case TransmissionType::FTM:
			m_currTransType = TransmissionType::DATA;
			m_ftmTxopQueue = std::queue<Ptr<Txop>>();
			Simulator::Schedule(m_timeTracker->GetDataTimeLength(), &CentralizedScheduler::SwitchTransType, this);
			break;
	
		default:
			NS_LOG_ERROR("TransmissionType should be either FTM or DATA!!");
	}

	if (IsTransmitting()) {
    return;
  }

  Ptr<Txop> candidateTxop = DequeueTransmission();
  if (candidateTxop != nullptr) {
    // WifiMacHeader hdr = candidateTxop->GetWifiMacQueue()->Peek()->GetHeader();
    // NS_LOG_DEBUG("Access dequeue, trans type: " << hdr.GetTypeString() << ", from=" << hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << ", time: " << Simulator::Now());

    Simulator::ScheduleNow(&Txop::CSRequestAccess, candidateTxop);
	}

	return;
}

void
CentralizedScheduler::TransmissionEnd()
{
	// Mark Transmission End, call scheduleNewTransmission()
	m_timeTracker->MarkTransmissionEndTime();
	m_preTransPairAddr.clear();

	std::copy(m_transPairAddr.begin(), m_transPairAddr.end(), std::back_inserter(m_preTransPairAddr));
	m_transPairAddr.clear();
	m_grantedTxop = nullptr;

	m_isTransmitting = false;

	Ptr<Txop> candidateTxop = DequeueTransmission();
	if (candidateTxop != nullptr) {
		
		NS_LOG_DEBUG("Transmission end dequeue, {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
							 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

		Simulator::ScheduleNow(&Txop::CSRequestAccess, candidateTxop);
	}
}

bool
CentralizedScheduler::GetTransmissionGranted(Ptr<Txop> in_txop)
{
	Ptr<WifiMacQueue> macQueue = in_txop->GetWifiMacQueue();
	WifiMacHeader hdr = macQueue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = macQueue->Peek()->GetItem()->GetPacket();
	TransmissionType type = GetTransmissionType (packet->Copy(), hdr);

	// if (hdr.GetAddr1().IsBroadcast()) {
	// 	return true; // just let it through
	// }
	
	if ((type != TransmissionType::ACK && type != m_currTransType && type != TransmissionType::MGT_OTHERS && !hdr.GetAddr1().IsBroadcast()) ||
			(m_grantedTxop == nullptr || in_txop != m_grantedTxop)) {
		return false;
	}

	NS_LOG_DEBUG("Get Transmit DATA Type: " << hdr.GetTypeString() << ", from=" << 
							 hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << ", {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
							 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	ReleaseLock();
	TransmissionStart();
	return true;
}

void
CentralizedScheduler::BeginTransmit(Ptr<Txop> in_txop)
{
	Ptr<WifiMacQueue> macQueue = in_txop->GetWifiMacQueue();
	WifiMacHeader hdr = macQueue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = macQueue->Peek()->GetItem()->GetPacket();

	NS_LOG_DEBUG("Get Transmit DATA Type: " << hdr.GetTypeString() << ", from=" << 
							 hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << ", {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
							 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());
	
	ReleaseLock();
	TransmissionStart();
}

void
CentralizedScheduler::ReleaseLock()
{
	m_lock = 0;
}

bool
CentralizedScheduler::GetLock()
{
	NS_LOG_DEBUG("m_lock value: " << m_lock << ", function belonging: " << this << ", time: " << Simulator::Now());
	if (m_lock == 0) {
		m_lock++;
		NS_LOG_DEBUG("Got lock, m_lock value: " << m_lock << ", function belonging: " << this << ", time: " << Simulator::Now());
		return true;
	} else {
		NS_LOG_DEBUG("Couldn't get lock, m_lock value: " << m_lock << ", function belonging: " << this << ", time: " << Simulator::Now());
		return false;
	}
}

bool
CentralizedScheduler::IsTransmitting()
{
	return m_isTransmitting;
}

TransmissionType
CentralizedScheduler::GetTransmissionType(Ptr<Packet> in_packet, WifiMacHeader in_hdr)
{
	if (in_hdr.IsAck() || in_hdr.IsBlockAck()) {
		return TransmissionType::ACK;
	}

	if (in_hdr.IsMgt() && in_hdr.IsAction()) {
		WifiActionHeader actionHdr;
		in_packet->PeekHeader(actionHdr);

		if (actionHdr.GetCategory() == WifiActionHeader::PUBLIC_ACTION) {
			return TransmissionType::FTM;
		} else {
			return TransmissionType::MGT_OTHERS;
		}
	} else if (in_hdr.IsMgt()) {
		return TransmissionType::MGT_OTHERS;
	}
	return TransmissionType::DATA;
}

bool
CentralizedScheduler::EnqueueTransmission(Ptr<Txop> in_txop)
{
	Ptr<WifiMacQueue> macQueue = in_txop->GetWifiMacQueue();

	if (macQueue->IsEmpty()) {
		return false;
	}

	WifiMacHeader hdr = macQueue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = macQueue->Peek()->GetItem()->GetPacket();
	TransmissionType type = GetTransmissionType (packet->Copy(), hdr);

	if (hdr.GetAddr1().IsBroadcast()) {
		m_broadcastQueue.push(in_txop);
	} else if (type == TransmissionType::DATA) {
		m_dataTxopQueue.push(in_txop);
	} else if (type == TransmissionType::FTM) {
		m_ftmTxopQueue.push(in_txop);
	} else if (type == TransmissionType::MGT_OTHERS) {
		m_mgtOtherQueue.push(in_txop);
	} else {
		NS_FATAL_ERROR("Unable to Enqueue, type not found");
	}
	
	NS_LOG_DEBUG("Enqueue, DATA Type: " << hdr.GetTypeString() << ", from=" << 
							 hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << ", {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
							 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	return true;

}

Ptr<Txop>
CentralizedScheduler::DequeueTransmission()
{
	if (!GetLock()) {
		return nullptr;
	}

	Ptr<Txop> candidateTxop = nullptr;
	int flag = 0;

	while (flag == 0) {
		if (!m_broadcastQueue.empty()) {
			candidateTxop = m_broadcastQueue.front();
			m_broadcastQueue.pop();
		} else if (!m_mgtOtherQueue.empty()) {
			candidateTxop = m_mgtOtherQueue.front();
			m_mgtOtherQueue.pop();
		} else if (m_currTransType == TransmissionType::DATA && !m_dataTxopQueue.empty()) {
			candidateTxop = m_dataTxopQueue.front();
			m_dataTxopQueue.pop();
		} else if (m_currTransType == TransmissionType::FTM && !m_ftmTxopQueue.empty()) {
			candidateTxop = m_ftmTxopQueue.front();
			m_ftmTxopQueue.pop();
		} else {
			NS_LOG_DEBUG("Centralized Scheduler queue empty.");
			ReleaseLock();
			return nullptr;
		}

		flag = 1;

		if (!candidateTxop->HasFramesToTransmit()) {
			flag = 0;
		}
	}
	
	
	Ptr<WifiMacQueue> macQueue = candidateTxop->GetWifiMacQueue();

	WifiMacHeader hdr = macQueue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = macQueue->Peek()->GetItem()->GetPacket();
	TransmissionType type = GetTransmissionType (packet->Copy(), hdr);

	m_transPairAddr.push_back(hdr.GetAddr1());
	m_transPairAddr.push_back(hdr.GetAddr2());

	m_grantedTxop = candidateTxop;

	NS_LOG_DEBUG("Dequeue, allow data type: " << m_currTransType << ", CS queue type: " << type << ", DATA Type: " <<
							 hdr.GetTypeString() << ", from=" << hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << 
							 ", {mQ_size, dQ_size, fQ_size, bQ_size}={" << m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	return candidateTxop;
}

Mac48Address
CentralizedScheduler::GetTransFromAddr()
{
	return m_transPairAddr[1];
}

Mac48Address
CentralizedScheduler::GetTransToAddr()
{
	return m_transPairAddr[0];
}

SchedulerPhyRxProxy::SchedulerPhyRxProxy(Ptr<CentralizedScheduler> in_scheduler, Mac48Address in_mac_addr)
{
	m_centralized_scheduler = in_scheduler;
	m_mac_addr = in_mac_addr;
}

SchedulerPhyRxProxy::~SchedulerPhyRxProxy()
{
	TraceDisconnectWithoutContext("PhyRxBegin", MakeCallback(&SchedulerPhyRxProxy::PhyRxBegin, this));
	m_centralized_scheduler = 0;
}

void
SchedulerPhyRxProxy::SetPhyRxCallBack(bool in_cs_enabled, Ptr<WifiPhy> in_phy)
{
	in_phy->TraceConnectWithoutContext("PhyRxBegin", MakeCallback(&SchedulerPhyRxProxy::PhyRxBegin, this));
}

void
SchedulerPhyRxProxy::PhyRxBegin(Ptr<const Packet> in_packet, RxPowerWattPerChannelBand rxPowersW)
{
	NS_LOG_FUNCTION (this);
	Ptr<Packet> cpy_packet = in_packet->Copy();

	WifiMacHeader hdr;
	cpy_packet->RemoveHeader(hdr);

	if (hdr.GetAddr1() != m_mac_addr) {
		return;
	}


	if (hdr.IsAck() || hdr.IsBlockAck()) {
		NS_LOG_DEBUG("PHY Got ACK that is not DATA, from: " << hdr.GetAddr2() << ", I'm: " << m_mac_addr << " time: " << Simulator::Now());
		m_centralized_scheduler->TransmissionEnd();
	}
}

} /* namespace ns3 */