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

#include <tuple>

namespace ns3 {

class TransmissionSelector : public Object
{
public:
	TransmissionSelector() {}
	virtual ~TransmissionSelector() {}
	virtual bool CheckPreambleDetectable() = 0;
	virtual void AddCandidatePairToList() = 0;
	virtual std::tuple<int,int> SelecFinalCandidate() = 0;
};

}

#endif /* TRANSMISSIONSELECTOR_H_ */

#ifndef FTMTRANSMISSIONSELECTOR_H
#define FTMTRANSMISSIONSELECTOR_H

namespace ns3 {

class FtmTransmissionSelector : public TransmissionSelector
{
public:
	FtmTransmissionSelector() {}
	virtual ~FtmTransmissionSelector() {}
	bool CheckPreambleDetectable();
	void AddCandidatePairToList();
	std::tuple<int,int> SelecFinalCandidate();
private:
	std::tuple<int,int> finalCandidate;
	std::vector<std::tuple<int,int>> candidatePairList;

};

}

#endif /* FTMTRANSMISSIONSELECTOR_H_ */

#ifndef DATATRANSMISSIONSELECTOR_H
#define DTATTRANSMISSIONSELECTOR_H

namespace ns3 {

class DataTransmissionSelector : public TransmissionSelector
{
public:
	DataTransmissionSelector() {}
	virtual ~DataTransmissionSelector() {}
	bool CheckPreambleDetectable();
	void AddCandidatePairToList();
	std::tuple<int,int> SelecFinalCandidate();
private:
	std::tuple<int,int> finalCandidate;
	std::vector<std::tuple<int,int>> candidatePairList;

};

} /* namespace ns3 */

#endif /* DTATTRANSMISSIONSELECTOR_H_ */