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

#include <vector>
#include <tuple>

namespace ns3 {

class CentralizedScheduler : public Object
{
public:
	CentralizedScheduler() {}
	virtual ~CentralizedScheduler() {}
	void StartTransmission();
	void SwitchTransmissionType();
	void TransmissionEnd(); // callback 
	void SelectFTMCandidate();
	void SelectDataCandidate();

private:
	int8_t m_currentTransmissionType;
	std::tuple<int, int> m_transmissionPair;
	std::vector<uint32_t> m_dataTransmittedByteList;
	std::vector<int> m_ftmMeasuredCountList;
	
};

} /* namespace ns3 */


#endif /* CENTRALIZEDSCHEDULER_H_ */