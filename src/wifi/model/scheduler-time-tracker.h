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

#ifndef SCHEDULERTIMETRACKER_H
#define SCHEDULERTIMETRACKER_H

#include "ns3/object.h"
#include "ns3/nstime.h"

namespace ns3 {

class SchedulerTimeTracker : public Object
{
public:
	SchedulerTimeTracker() {}
	virtual ~SchedulerTimeTracker() {}
	bool ReachFtmProportionTime();
	bool ReachTimeAllocationPeriod();
	Time MarkTransmissionStart();
	Time MarkTransmissionEnd();
private:
	Time m_timeAllocationPeriod;
	Time m_transmissionStartTime;
	Time m_transmissionEndTime;
	Time m_ftmProportionTime;
	Time periodStartTime;
	double alpha;

};
	
} /* namespace ns3 */

#endif /* SCHEDULERTIMETRACKER_H_ */