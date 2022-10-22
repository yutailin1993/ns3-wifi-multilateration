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

#ifndef TRANSMISSIONSELECTOR_H
#define TRANSMISSIONSELECTOR_H

#include "ns3/object.h"
#include "ns3/mac48-address.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-phy.h"
#include "ns3/packet.h"

#include <tuple>
#include <map>

namespace ns3 {

class TransmissionSelector : public Object
{
public:
	TransmissionSelector();
	virtual ~TransmissionSelector();
	void SetCandidate(WifiMacHeader in_hdr);
	std::vector<Mac48Address> GetCandidateAddrs();
	void ResetCandidateAddrs();
	void RegisterDevice(int in_deviceNo, Mac48Address in_deviceAddr);
	void ReleaseLock();
	bool GetLock();

private:
	int GetIndexByAddr(Mac48Address in_addr);
	int m_lock;

	std::map<const int, Mac48Address> m_nodeAddrTable;
	std::vector<Mac48Address> m_candidateAddrs;

};

} /* namespace ns3 */

#endif /* TRANSMISSIONSELECTOR_H_ */