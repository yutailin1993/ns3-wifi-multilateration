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
#include <map>
#include <list>
#include <cassert>

namespace ns3 {

class Txop;
class ChannelAccessManager;

NS_LOG_COMPONENT_DEFINE ("CentralizedScheduler");

NS_OBJECT_ENSURE_REGISTERED (CentralizedScheduler);

CentralizedScheduler::CentralizedScheduler(double in_timeTrackerAlpha,
																					 Time in_timeTrackerPeriodLength,
																					 TransmissionType in_startTransType,
																					 std::vector<std::vector<int>> in_independentSets)
{
	// initialize time tracker and transmission selector
	m_timeTracker = CreateObject<SchedulerTimeTracker> (in_timeTrackerAlpha, in_timeTrackerPeriodLength);
	m_currTransType = in_startTransType;
	m_isTransmitting = false;
	m_lock = 0;
	m_ongoingTransmissionCnt = 0;
	m_scheduledTransmissionCnt = 0;
	m_currentIndependentSetIdx = 0;

	m_independentSets = in_independentSets;

	if (in_timeTrackerAlpha != 0) {
		int64_t ftmDurationInt = m_timeTracker->GetFtmTimeLength().GetPicoSeconds();
		Simulator::Schedule(PicoSeconds(ftmDurationInt-1), &CentralizedScheduler::SwitchTransType, this);
	}
}

CentralizedScheduler::~CentralizedScheduler()
{
	m_timeTracker = 0;
	m_mgtOtherQueue = std::list<Ptr<Txop>>();
	m_dataTxopQueue = std::list<Ptr<Txop>>();
	m_ftmTxopQueue = std::list<Ptr<Txop>>();
	m_broadcastQueue = std::list<Ptr<Txop>>();
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
			m_ftmTxopQueue.clear();
			Simulator::Schedule(m_timeTracker->GetDataTimeLength(), &CentralizedScheduler::SwitchTransType, this);
			break;
	
		default:
			NS_LOG_ERROR("TransmissionType should be either FTM or DATA!!");
	}

	if (IsTransmitting()) {
    return;
  }

	ScheduleNextTransmission();

	return;
}

std::vector<int>
CentralizedScheduler::GetNextIndependentSet()
{
	bool checked = false;
	while (!checked) {
		if (m_ftmTxopQueue.empty()) {
			break;
		}
		Ptr<const WifiMacQueueItem> candidateItem = m_ftmTxopQueue.front()->GetWifiMacQueue()->Peek();
		if (candidateItem == 0) {
			m_ftmTxopQueue.pop_front();
			continue;
		}
		checked = true;
	}

	if (m_ftmTxopQueue.empty()) {
		std::vector<int> n = {-1};
		return n;
	}

	const Ptr<Txop> fristFTMTxop = m_ftmTxopQueue.front();
	Mac48Address senderAddr = fristFTMTxop->GetWifiMacQueue()->Peek()->GetItem()->GetHeader().GetAddr2();
	Mac48Address receiverAddr = fristFTMTxop->GetWifiMacQueue()->Peek()->GetItem()->GetHeader().GetAddr1();
	
	bool found = false;
	int start = m_currentIndependentSetIdx;
	while (!found) {
		std::vector<int> set = m_independentSets[m_currentIndependentSetIdx];
		for (int i=0; i<set.size(); i+= 2) {
			Mac48Address candidateAddr = GetAddressBySTAIndex(set[i]);
			Mac48Address candidatePeerAddr = GetAddressBySTAIndex(set[i+1]);
			if (((senderAddr == candidateAddr && receiverAddr == candidatePeerAddr) ||
				(senderAddr == candidatePeerAddr && receiverAddr == candidateAddr))) {
					found = true;
					break;
				}
		}
		if (found) {
			break;
		}
		m_currentIndependentSetIdx = (m_currentIndependentSetIdx + 1) % m_independentSets.size();
		if (m_currentIndependentSetIdx == start) {
			NS_FATAL_ERROR("Iterate Through independent set but couldn't found any matches!!");
		}
	}

	int k = m_currentIndependentSetIdx;
	m_currentIndependentSetIdx = (m_currentIndependentSetIdx + 1) % m_independentSets.size();

	return m_independentSets[k];
}

void
CentralizedScheduler::ScheduleNextTransmission()
{
	std::vector<Ptr<Txop>> candidateTxops;
	if (m_currTransType == TransmissionType::DATA) {
		candidateTxops.push_back(DataDequeueTransmission());
	} else {
		candidateTxops = FtmDequeueTransmission();
	}

	for (auto &txop : candidateTxops) {
		if (txop != nullptr) {
			NS_LOG_DEBUG("Transmission end dequeue, {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
						 				 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
						 				 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());
			Simulator::ScheduleNow(&Txop::CSRequestAccess, txop);
			IncreaseScheduleTransCounter();
		}
	}
}

void
CentralizedScheduler::IncreaseScheduleTransCounter()
{
	m_scheduledTransmissionCnt += 1;
}

void
CentralizedScheduler::DecreaseScheduleTransCounter()
{
	m_scheduledTransmissionCnt -= 1;
	assert (m_scheduledTransmissionCnt >= 0);
}

void
CentralizedScheduler::IncreaseOngoingTransCounter()
{
	m_ongoingTransmissionCnt += 1;
	NS_LOG_DEBUG("Increase ong trans current: " << m_ongoingTransmissionCnt);
}

void
CentralizedScheduler::DecreaseOngoingTransCounter()
{
	m_ongoingTransmissionCnt -= 1;
	NS_LOG_DEBUG("Decrease ong trans current: " << m_ongoingTransmissionCnt);
	if (m_ongoingTransmissionCnt < 0) {
		NS_LOG_DEBUG("Oh no, not acceptable!!");
		m_ongoingTransmissionCnt = 0;
	}
	assert (m_ongoingTransmissionCnt >= 0);
}

bool
CentralizedScheduler::IsAllTransmissionScheduled()
{
	if (m_scheduledTransmissionCnt == 0) {
		return true;
	}
	return false;
}

bool
CentralizedScheduler::IsAllTransmissionEnd()
{
	if (m_ongoingTransmissionCnt == 0) {
		return true;
	}
	return false;
}

void
CentralizedScheduler::TransmissionEnd()
{
	// Mark Transmission End, call scheduleNewTransmission()
	DecreaseOngoingTransCounter();
	if (!IsAllTransmissionEnd()) {
		return;
	}
	m_timeTracker->MarkTransmissionEndTime();
	m_isTransmitting = false;

	ScheduleNextTransmission();
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
	
	DecreaseScheduleTransCounter();
	IncreaseOngoingTransCounter();

	if (IsAllTransmissionScheduled()) {
		ReleaseLock();
	}

	if (m_ongoingTransmissionCnt == 1) {
		TransmissionStart();
	}
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
		m_broadcastQueue.push_back(in_txop);
	} else if (type == TransmissionType::DATA) {
		m_dataTxopQueue.push_back(in_txop);
	} else if (type == TransmissionType::FTM) {
		m_ftmTxopQueue.push_back(in_txop);
	} else if (type == TransmissionType::MGT_OTHERS) {
		m_mgtOtherQueue.push_back(in_txop);
	} else {
		NS_FATAL_ERROR("Unable to Enqueue, type not found");
	}
	
	NS_LOG_DEBUG("Enqueue, DATA Type: " << hdr.GetTypeString() << ", from=" << 
							 hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << ", {mQ_size, dQ_size, fQ_size, bQ_size}={" <<
							 m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	return true;

}

void
CentralizedScheduler::ConstructStaAddrTable(std::vector<Address> in_staAddrs)
{
	for (int i=0; i<in_staAddrs.size(); i++) {
		Mac48Address addr = Mac48Address::ConvertFrom(in_staAddrs[i]);
		InsertOrAssignAddress(i, addr);
	}
}

void
CentralizedScheduler::InsertOrAssignAddress(int in_index, Mac48Address in_macAddr)
{
	m_staMacAddresses.insert_or_assign(in_index, in_macAddr);
}

Mac48Address
CentralizedScheduler::GetAddressBySTAIndex(int in_index)
{
	if (m_staMacAddresses.find(in_index) == m_staMacAddresses.end()) {
		NS_FATAL_ERROR("Key " << in_index << " not found in STA mac addresses list!!");
	}

	return m_staMacAddresses[in_index];
}

TransmissionType
CentralizedScheduler::GetCurrentGrantedTransmissionType()
{
	return m_currTransType;
}

Ptr<Txop>
CentralizedScheduler::FtmFindCandidate(int in_candidate, int in_candidatePeer)
{
	Ptr<Txop> candidateTxop;
	int flag = 0;
	while (flag == 0) {
		if (m_currTransType == TransmissionType::FTM && !m_ftmTxopQueue.empty()) {
			candidateTxop = GetFtmCandidateTxop(in_candidate, in_candidatePeer);
		}

		if (candidateTxop == nullptr) {
			NS_LOG_DEBUG("Cannot find FTM txop with candidate " << in_candidate << " and peer " << in_candidatePeer);
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

	NS_LOG_DEBUG("Dequeue, allow data type: " << m_currTransType << ", CS queue type: " << type << ", DATA Type: " <<
							 hdr.GetTypeString() << ", from=" << hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << 
							 ", {mQ_size, dQ_size, fQ_size, bQ_size}={" << m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	return candidateTxop;
}

std::vector<Ptr<Txop>>
CentralizedScheduler::FtmDequeueTransmission()
{
	std::vector<Ptr<Txop>> candidateTxops;
	if (!GetLock()) {
		candidateTxops.push_back(nullptr);
		return candidateTxops;
	}

	// 1D vector with multiple peer links i.e. [candidate1, candidatePeer1, candidate2, candidatePeer2, ...]
	std::vector<int> independentSetNodes;
	independentSetNodes = GetNextIndependentSet();
	if (independentSetNodes[0] == -1) {
		candidateTxops.push_back(nullptr);
		ReleaseLock();
		return candidateTxops;
	}

	for (int i=0; i<independentSetNodes.size(); i+=2) {
		int candidateId = independentSetNodes[i];
		int candidatePeerId = independentSetNodes[i+1];
		Ptr<Txop> candidateTxop = FtmFindCandidate(candidateId, candidatePeerId);
		if (candidateTxop == nullptr) {
			continue;
		}
		candidateTxops.push_back(candidateTxop);
	}

	if (candidateTxops.size() == 0) {
		candidateTxops.push_back(nullptr);
		ReleaseLock();
	}

	return candidateTxops;
}

Ptr<Txop>
CentralizedScheduler::GetFtmCandidateTxop(int in_candidate, int in_candidate_peer)
{
	Ptr<Txop> candidateTxop = nullptr;
	Mac48Address candidateAddr = GetAddressBySTAIndex(in_candidate);
	Mac48Address candidatePeerAddr = GetAddressBySTAIndex(in_candidate_peer);

	std::list<Ptr<Txop>>::iterator iter = m_ftmTxopQueue.begin();

	while (iter != m_ftmTxopQueue.end()) {
		Ptr<const WifiMacQueueItem> candidateItem = (*iter)->GetWifiMacQueue()->Peek();
		if (candidateItem == 0) {
			m_ftmTxopQueue.erase(iter++);
			continue;
		}
		Mac48Address senderAddr = candidateItem->GetItem()->GetHeader().GetAddr2();
		Mac48Address receiverAddr = candidateItem->GetItem()->GetHeader().GetAddr1();
		if ((senderAddr == candidateAddr && receiverAddr == candidatePeerAddr) ||
				(senderAddr == candidatePeerAddr && receiverAddr == candidateAddr)) {
			candidateTxop = *iter;
			m_ftmTxopQueue.erase(iter++);
			break;
		} else {
			++iter;
		}
	}

	return candidateTxop;
}

Ptr<Txop>
CentralizedScheduler::DataDequeueTransmission()
{
	if (!GetLock()) {
		return nullptr;
	}

	Ptr<Txop> candidateTxop = nullptr;
	int flag = 0;

	while (flag == 0) {
		if (!m_broadcastQueue.empty()) {
			candidateTxop = m_broadcastQueue.front();
			m_broadcastQueue.pop_front();
		} else if (!m_mgtOtherQueue.empty()) {
			candidateTxop = m_mgtOtherQueue.front();
			m_mgtOtherQueue.pop_front();
		} else if (m_currTransType == TransmissionType::DATA && !m_dataTxopQueue.empty()) {
			candidateTxop = m_dataTxopQueue.front();
			m_dataTxopQueue.pop_front();
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

	NS_LOG_DEBUG("Dequeue, allow data type: " << m_currTransType << ", CS queue type: " << type << ", DATA Type: " <<
							 hdr.GetTypeString() << ", from=" << hdr.GetAddr2() << ", to=" << hdr.GetAddr1() << 
							 ", {mQ_size, dQ_size, fQ_size, bQ_size}={" << m_mgtOtherQueue.size() << "," << m_dataTxopQueue.size() << "," << 
							 m_ftmTxopQueue.size() << "," << m_broadcastQueue.size() << "}, time: " << Simulator::Now());

	return candidateTxop;
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
