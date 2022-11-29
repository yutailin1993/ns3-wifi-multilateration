
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
#include "ns3/nstime.h"

#include "scheduler-time-tracker.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SchedulerTimeTracker");

NS_OBJECT_ENSURE_REGISTERED (SchedulerTimeTracker);

SchedulerTimeTracker::SchedulerTimeTracker(double in_alpha, Time in_allocationPeriodLength)
{
	m_alpha = in_alpha;
	m_allocationPeriodLength = in_allocationPeriodLength;
	m_ftmTimeLength = in_allocationPeriodLength * in_alpha;
	m_dataTimeLength = m_allocationPeriodLength * (1-in_alpha);
	m_ftmEndTime = in_allocationPeriodLength * in_alpha;
	m_periodEndTime = in_allocationPeriodLength;
	m_periodStartTime = MilliSeconds (0);
	m_transmissionEndTime = MilliSeconds (0);
	m_transmissionStartTime = MilliSeconds (0);
}

SchedulerTimeTracker::~SchedulerTimeTracker()
{
	m_alpha = 0;
	m_allocationPeriodLength = MilliSeconds(0);
	m_ftmEndTime = MilliSeconds (0);
	m_periodEndTime = MilliSeconds (0);
	m_periodStartTime = MilliSeconds (0);
	m_transmissionEndTime = MilliSeconds (0);
	m_transmissionStartTime = MilliSeconds (0);
	m_ftmTimeLength = MilliSeconds (0);
	m_dataTimeLength = MilliSeconds (0);		
}

void
SchedulerTimeTracker::MarkTransmissionStartTime()
{
	m_transmissionStartTime = Simulator::Now();
}

void
SchedulerTimeTracker::MarkTransmissionEndTime()
{
	m_transmissionEndTime = Simulator::Now();
}

Time
SchedulerTimeTracker::GetTransmissionStartTime()
{
	return m_transmissionStartTime;
}

Time
SchedulerTimeTracker::GetTransmissionEndTime()
{
	return m_transmissionEndTime;
}

Time
SchedulerTimeTracker::GetFtmEndTime()
{
	return m_ftmEndTime;
}

Time
SchedulerTimeTracker::GetPeriodEndTime()
{
	return m_periodEndTime;
}

Time
SchedulerTimeTracker::GetFtmTimeLength()
{
	return m_ftmTimeLength;
}

Time
SchedulerTimeTracker::GetDataTimeLength()
{
	return m_dataTimeLength;
}

double
SchedulerTimeTracker::GetAlpha()
{
	return m_alpha;
}

void
SchedulerTimeTracker::UpdateTimes()
{
	Time now = Simulator::Now();
	m_periodStartTime = now;
	m_periodEndTime = now + m_allocationPeriodLength;
	m_ftmEndTime = now + m_allocationPeriodLength*m_alpha;
}

void
SchedulerTimeTracker::UpdateAlpha(double in_alpha)
{
	NS_ASSERT_MSG((in_alpha >= 0.0 && in_alpha <= 1.0), "Alpha should within 0 to 1!!");
	m_alpha = in_alpha;
}

} /* namespace ns3 */