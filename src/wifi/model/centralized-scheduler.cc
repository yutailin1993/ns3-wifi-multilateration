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
#include "centralized-scheduler.h"
#include "scheduler-time-tracker.h"
#include "transmission-selector.h"

#include <vector>
#include <tuple>

namespace ns3 {

class Txop;

NS_LOG_COMPONENT_DEFINE ("CentralizedScheduler");

NS_OBJECT_ENSURE_REGISTERED (CentralizedScheduler);

CentralizedScheduler::CentralizedScheduler(double in_timeTrackerAlpha, Time in_timeTrackerPeriodLength, TransmissionType in_startTransType)
{
	// initialize time tracker and transmission selector
	m_transSelector = CreateObject<TransmissionSelector> ();
	m_timeTracker = CreateObject<SchedulerTimeTracker> (in_timeTrackerAlpha, in_timeTrackerPeriodLength);
	m_currTransType = in_startTransType;
	uint8_t broadcastAddr[6] = {255, 255, 255, 255, 255, 255};
	m_broadcastAddr.CopyFrom(broadcastAddr);

	if (in_timeTrackerAlpha != 0) {
		Simulator::Schedule(m_timeTracker->GetFtmTimeLength(), &CentralizedScheduler::SwitchTransType, this);
	}
}

CentralizedScheduler::~CentralizedScheduler()
{
	m_transSelector = 0;
	m_timeTracker = 0;
	m_preTransPairAddr.clear();
	m_transPairAddr.clear();
}

void
CentralizedScheduler::DoDispose(void)
{
	NS_LOG_FUNCTION(this);
}

void
CentralizedScheduler::TransmissionStart()
{
	m_timeTracker->MarkTransmissionStartTime();

}

void
CentralizedScheduler::SwitchTransType()
{
	switch (m_currTransType)
	{
		case TransmissionType::DATA:
			m_currTransType = TransmissionType::FTM;
			Simulator::Schedule(m_timeTracker->GetFtmTimeLength(), &CentralizedScheduler::SwitchTransType, this);
			return;

		case TransmissionType::FTM:
			m_currTransType = TransmissionType::DATA;
			Simulator::Schedule(m_timeTracker->GetDataTimeLength(), &CentralizedScheduler::SwitchTransType, this);
			return;
	
		default:
			NS_LOG_ERROR("TransmissionType should be either FTM or DATA!!");
	}
}

bool
CentralizedScheduler::TransmissionEnd()
{
	// Mark Transmission End, call scheduleNewTransmission()
	m_timeTracker->MarkTransmissionEndTime();
	m_preTransPairAddr.clear();

	std::copy(m_transPairAddr.begin(), m_transPairAddr.end(), std::back_inserter(m_preTransPairAddr));
	m_transPairAddr.clear();

	m_transSelector->ResetCandidateAddrs();
	m_transSelector->ReleaseLock();

	// std::cout << "Transmission End at time: " << m_timeTracker->GetTransmissionEndTime() << std::endl;

	return true;
}

bool
CentralizedScheduler::GetTransmissionGranted(Ptr<Txop> in_txop)
{
	Ptr<WifiMacQueue> queue = in_txop->GetWifiMacQueue();

	if (queue->IsEmpty()) {
		return false;
	}

	WifiMacHeader hdr = queue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = queue->Peek()->GetItem()->GetPacket();
	TransmissionType type = GetTransmissionType (packet->Copy(), hdr);

	if (hdr.GetAddr1() == m_broadcastAddr) {
		return true; // just let it through
	}
	
	if (type != TransmissionType::ACK && type != m_currTransType) {
		return false;
	}

	if (m_transPairAddr[0] != hdr.GetAddr1() || m_transPairAddr[1] != hdr.GetAddr2()) { // not granted transmission pair
		// uint8_t a[6], b[6];
		// hdr.GetAddr2().CopyTo(a);
		// hdr.GetAddr1().CopyTo(b);
		// std::cout << "Contend fail" << " Trans Addr: " << int(a[5]) << ", Recv Addr: " << int(b[5]) << " Time: " << Simulator::Now() << std::endl;
		return false;
	}

	/* debug */
	uint8_t a[6], b[6];
	hdr.GetAddr2().CopyTo(a);
	hdr.GetAddr1().CopyTo(b);


	// std::cout << "Sender Addr: " << int(a[5]) << " Receiver Addr: " << int(b[5]) << ", Time: " << Simulator::Now() << std::endl;

	/* end debug */

	return true;
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
		}
	}
	return TransmissionType::DATA;
}

bool
CentralizedScheduler::ContendForTransmissionGrant(Ptr<Txop> in_txop)
{
	Ptr<WifiMacQueue> queue = in_txop->GetWifiMacQueue();

	if (queue->IsEmpty()) {
		return false;
	}

	WifiMacHeader hdr = queue->Peek()->GetItem()->GetHeader();
	Ptr<const Packet> packet = queue->Peek()->GetItem()->GetPacket();
	TransmissionType type = GetTransmissionType (packet->Copy(), hdr);

	if (hdr.GetAddr1() == m_broadcastAddr) {
		return false;
	}

	if (type != m_currTransType) {
		return false;
	}

	/* debug */
	uint8_t a[6], b[6];
	hdr.GetAddr1().CopyTo(a);
	hdr.GetAddr2().CopyTo(b);

	/* end debug */

	if (std::find(m_preTransPairAddr.begin(), m_preTransPairAddr.end(), hdr.GetAddr1()) != m_preTransPairAddr.end() &&
			std::find(m_preTransPairAddr.begin(), m_preTransPairAddr.end(), hdr.GetAddr2()) != m_preTransPairAddr.end()) { // not granted transmission pair
		return false;
	}

	if (m_transSelector->GetLock()) {
		m_transSelector->SetCandidate(hdr);
		std::vector<Mac48Address> candidateAddrs = m_transSelector->GetCandidateAddrs();
		for (auto &addrIter : candidateAddrs) {
			m_transPairAddr.push_back(addrIter);
		}

		// std::cout << "Trans Type: " << int(m_currTransType) << " Contend Trans: " << int(b[5]) << ", Recv: " << int(a[5]) << ", Success" << std::endl;
		return true;
	}

	return false;
}

void
CentralizedScheduler::RegisterDevice(int in_deviceNo, Mac48Address in_deviceAddr)
{
	m_transSelector->RegisterDevice(in_deviceNo, in_deviceAddr);
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

	/* debug */
	uint8_t a[6], b[6], c[6];

	m_mac_addr.CopyTo(a);
	hdr.GetAddr1().CopyTo(b);
	hdr.GetAddr2().CopyTo(c);

	if (hdr.IsMgt() && hdr.IsAction()) {
		WifiActionHeader actionHdr;
		cpy_packet->RemoveHeader(actionHdr);

		if (actionHdr.GetCategory() == WifiActionHeader::PUBLIC_ACTION) {
			if (actionHdr.GetAction().publicAction == WifiActionHeader::FTM_REQUEST) {
				// std::cout << int (a[5]) << " Got FTM_REQUEST" << " From " << int(c[5]) << ", To " << int(b[5]) << " Time: " << Simulator::Now() << std::endl;
			} else if (actionHdr.GetAction().publicAction == WifiActionHeader::FTM_RESPONSE) {
				// std::cout << int (a[5]) << " Got FTM_RESPONSE" << " From " << int(c[5]) << ", To " << int(b[5])<< " Time: " << Simulator::Now() << std::endl;
			}
		}
	} else if (hdr.IsData()) {
		// std::cout << int (a[5]) << "Got data" << " From " << int(c[5]) << ", To " << int(b[5]) << " Time: " << Simulator::Now() << std::endl; 
	}

	if (hdr.IsAck() || hdr.IsBlockAck()) {
		// std::cout << int (a[5]) << " Got ACK" << " From " << int(c[5]) << ", To " << int(b[5])<< " Time: " << Simulator::Now() << std::endl;
	}
	/* end debug*/

	if (hdr.IsAck() || hdr.IsBlockAck()) {
		if (hdr.GetAddr1() == m_mac_addr) {
			m_centralized_scheduler->TransmissionEnd();
		}
	}
}

} /* namespace ns3 */