/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
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

#ifndef CENTRALIZEDSCHEDULER_H
#define CENTRALIZEDSCHEDULER_H

#include "ns3/object.h"
#include "ns3/mac48-address.h"
#include "ns3/wifi-phy.h"

#include "txop.h"
#include "transmission-selector.h"
#include "scheduler-time-tracker.h"

#include <vector>
#include <tuple>
#include <queue>

namespace ns3 {

enum TransmissionType
{
	FTM,
	DATA,
	MGT_OTHERS,
	ACK // Note that m_currTransType should never be ACK in any circumstance
};

class CentralizedScheduler : public Object
{
public:
	CentralizedScheduler(double in_timeTrackerAlpha,
											 Time in_timeTrackerPeriodLength,
											 TransmissionType in_startTransType);
	virtual ~CentralizedScheduler();
	void TransmissionStart();
	void TransmissionEnd();
	void BeginTransmit(Ptr<Txop> in_txop);
	bool GetTransmissionGranted(Ptr<Txop> in_txop);
	bool IsTransmitting();
	bool EnqueueTransmission(Ptr<Txop> in_txop);
	void ReleaseLock();
	Ptr<Txop> DequeueTransmission();
	Mac48Address GetTransToAddr();
	Mac48Address GetTransFromAddr();

protected:
	void DoDispose (void) override;
	bool GetLock();

private:
	TransmissionType GetTransmissionType(Ptr<Packet> in_packet, WifiMacHeader in_hdr);
	void SwitchTransType();

	Mac48Address m_ftmRequestAddr; // The address that request FTM
	Mac48Address m_ftmResponseAddr; // The address that response FTM
	Ptr<SchedulerTimeTracker> m_timeTracker;
	TransmissionType m_currTransType;
	Ptr<Txop> m_grantedTxop;
	std::vector<Mac48Address> m_transPairAddr; // idx 0 is To addr, idx 1 is From Addr
	std::vector<Mac48Address> m_preTransPairAddr;
	std::queue<Ptr<Txop>> m_ftmTxopQueue;
	std::queue<Ptr<Txop>> m_dataTxopQueue;
	std::queue<Ptr<Txop>> m_mgtOtherQueue;
	std::queue<Ptr<Txop>> m_broadcastQueue;
	bool m_isTransmitting;
	int m_lock;
	
};

} /* namespace ns3 */


#endif /* CENTRALIZEDSCHEDULER_H_ */

#ifndef SCHEDULERPHYRXPROXY_H
#define SCHEDULERPHYRXPROXY_H

namespace ns3 {
class SchedulerPhyRxProxy : public Object
{
public:
	SchedulerPhyRxProxy(Ptr<CentralizedScheduler> in_scheduler, Mac48Address in_mac_addr);
	virtual ~SchedulerPhyRxProxy();
	void SetPhyRxCallBack(bool in_cs_enabled, Ptr<WifiPhy> in_phy);

private:
	void PhyRxBegin(Ptr<const Packet> packet, RxPowerWattPerChannelBand rxPowersW);
	Ptr<CentralizedScheduler> m_centralized_scheduler;
	Mac48Address m_mac_addr;

};

} /* namespace ns3 */

#endif /* SCHEDULERPHYRXPROXY_H */