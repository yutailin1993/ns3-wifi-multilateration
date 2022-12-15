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

#include "ns3/core-module.h"
#include "ns3/mac48-address.h"
#include "ns3/wifi-mac-header.h"

#include "transmission-selector.h"
#include "centralized-scheduler.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TransmissionScheduler");

NS_OBJECT_ENSURE_REGISTERED (TransmissionSelector);

TransmissionSelector::TransmissionSelector()
{
	m_candidateAddrs.clear();
}

TransmissionSelector::~TransmissionSelector()
{
	m_candidateAddrs.clear();
}

std::vector<Mac48Address>
TransmissionSelector::GetCandidateAddrs()
{
	return m_candidateAddrs;
}

void
TransmissionSelector::ResetCandidateAddrs()
{
	m_candidateAddrs.clear();
}

void
TransmissionSelector::RegisterDevice(int in_deviceNo, Mac48Address in_deviceAddr)
{
	m_nodeAddrTable.insert(std::pair<const int, Mac48Address> (in_deviceNo, in_deviceAddr));
}

void
TransmissionSelector::SetCandidate(WifiMacHeader in_hdr)
{
	Mac48Address toAddr, fromAddr;
	toAddr = in_hdr.GetAddr1();
	fromAddr = in_hdr.GetAddr2();

	m_candidateAddrs.push_back(toAddr);
	m_candidateAddrs.push_back(fromAddr);
}

int
TransmissionSelector::GetIndexByAddr(Mac48Address in_addr)
{
	for (auto iter=m_nodeAddrTable.begin(); iter!=m_nodeAddrTable.end(); iter++) {
		if (iter->second == in_addr) {
			return iter->first;
		}
	}

	NS_FATAL_ERROR("Address is not in the table, unacceptable!!");

	return -1;
}

} /* namespace ns3 */