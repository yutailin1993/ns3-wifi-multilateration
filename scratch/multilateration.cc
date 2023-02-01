#include <cmath>
#include <vector>
#include <set>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/ipv4-global-routing-helper.h"

#include "multilateration-core.h"

using namespace ns3;

/**
 * Element sequence in Matrix:
 *  | 0 1 |
 *  | 2 3 |
 */
typedef std::tuple<double, double, double, double> MatrixForm2D;

std::string
exec(const std::string cmd)
{
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
	if (!pipe) {
		throw std::runtime_error("popen() failed");
	}
	while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
		result += buffer.data();
	}

	return result;
}

template <class T>
std::ostringstream
ConstructStringMatrix(std::vector<std::vector<T>> in_matrixData)
{
	std::ostringstream strMatrixData;
	strMatrixData << "[";
	for (const auto &row : in_matrixData) {
		strMatrixData << "[";
		for (const auto &element : row) {
			strMatrixData << element;
			if (&element != &row.back()) {
				strMatrixData << ",";
			}
		}
		strMatrixData << "]";
		if (&row != &in_matrixData.back()) {
			strMatrixData << ",";
		}
	}
	strMatrixData << "]";

	return strMatrixData;
}

std::vector<std::vector<double>>
PositionListToVectorList(PositionList in_Positions)
{
	std::vector<std::vector<double>> positionVectors;
	for (const auto &pos : in_Positions) {
		positionVectors.push_back({std::get<0>(pos), std::get<1>(pos)});
	}
	return positionVectors;
}

std::vector<std::vector<int>>
GetIndependentSets(PositionList sta_positions)
{
	std::vector<std::vector<double>> positionsList = PositionListToVectorList(sta_positions);

	std::ostringstream strMatrixData = ConstructStringMatrix<double>(positionsList);

	std::string command = "python3 graph.py --idSets " + strMatrixData.str();
	std::string output = exec(command);

	std::vector<std::vector<int>> node_lists;
	std::istringstream outputStream(output);
	std::string line;
	while (std::getline(outputStream, line)) {
		std::istringstream lineStream(line);
		std::vector<int> row;
		int element;
		while (lineStream >> element) {
			row.push_back(element);
		}
		node_lists.push_back(row);
	}

	return node_lists;
}

std::set<std::vector<int>>
GetPeerLinksFromIndependentSets(std::vector<std::vector<int>> in_independentSets)
{
	std::set<std::vector<int>> peerLinks;
	for (auto setList : in_independentSets) {
		for (int i=0; i<setList.size(); i+=2) {
			peerLinks.insert({setList[i], setList[i+1]});
		}
	}

	return peerLinks;
}

MatrixForm2D
InverseMatrix(MatrixForm2D in_matrix)
{
	double e0 = std::get<0>(in_matrix);
	double e1 = std::get<1>(in_matrix);
	double e2 = std::get<2>(in_matrix);
	double e3 = std::get<3>(in_matrix);
	double m = 1.0 / ( e0*e3 - e1*e2 );

	MatrixForm2D matrix = {m*e3, m*(-e1),
												 m*(-e2), m*e0};
	return matrix;
}

MatrixForm2D
TransposeMatrix(MatrixForm2D in_matrix)
{
	double e0 = std::get<0>(in_matrix);
	double e1 = std::get<1>(in_matrix);
	double e2 = std::get<2>(in_matrix);
	double e3 = std::get<3>(in_matrix);

	MatrixForm2D matrix = {e0, e2,
												 e1, e3};
	return matrix;
}

MatrixForm2D
MultiplyMatrix(MatrixForm2D m1, MatrixForm2D m2)
{
	MatrixForm2D matrix = {
		std::get<0>(m1)*std::get<0>(m2)+std::get<1>(m1)*std::get<2>(m2), std::get<0>(m1)*std::get<1>(m2)+std::get<1>(m1)*std::get<3>(m2),
		std::get<2>(m1)*std::get<0>(m2)+std::get<3>(m1)*std::get<2>(m2), std::get<2>(m1)*std::get<1>(m2)+std::get<3>(m1)*std::get<3>(m2)
	};

	return matrix;
}

Position
LeastSquareError2D(PositionList apPosList, double distArray[])
{
	Position ap0 = apPosList[0];
	Position ap1 = apPosList[1];
	Position ap2 = apPosList[2];

	MatrixForm2D A = {
		2*std::get<0>(ap0)-2*std::get<0>(ap1), 2*std::get<1>(ap0)-2*std::get<1>(ap1),
		2*std::get<0>(ap0)-2*std::get<0>(ap2), 2*std::get<1>(ap0)-2*std::get<1>(ap2)
	};

	std::tuple<double, double> L = {
		pow(distArray[1], 2)-pow(distArray[0], 2)-(pow(std::get<0>(ap1), 2)-pow(std::get<0>(ap0), 2))-(pow(std::get<1>(ap1), 2)-pow(std::get<1>(ap0), 2)),
		pow(distArray[2], 2)-pow(distArray[0], 2)-(pow(std::get<0>(ap2), 2)-pow(std::get<0>(ap0), 2))-(pow(std::get<1>(ap2), 2)-pow(std::get<1>(ap0), 2))
	};

	MatrixForm2D ATrans = TransposeMatrix(A);
	MatrixForm2D tempResult = MultiplyMatrix(InverseMatrix(MultiplyMatrix(ATrans, A)), ATrans);
	std::tuple<double, double> result = {
		std::get<0>(tempResult)*std::get<0>(L)+std::get<1>(tempResult)*std::get<1>(L),
		std::get<2>(tempResult)*std::get<0>(L)+std::get<3>(tempResult)*std::get<1>(L)
	};

	return {std::get<0>(result), std::get<1>(result), 0};
}

std::vector<std::vector<double>>
GetEDM(EnvConfig in_envConf)
{
	// initialize
	std::vector<std::vector<double>> EDM;
	for (int i=0; i<in_envConf.nSTAs; i++) {
		std::vector<double> distVec(in_envConf.nSTAs, 0.0);
		EDM.push_back(distVec);
	}

	// get real p2p distance
	for (int i=0; i<in_envConf.nSTAs; i++) {
		for (int j=i+1; j<in_envConf.nSTAs; j++) {
			auto activeDistTpl = std::find_if(activeDistList.begin(), activeDistList.end(),
														[j, i](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == j && std::get<1>(e) == i);}
														);
			auto passiveDistTpl = std::find_if(passiveDistList.begin(), passiveDistList.end(),
														[j, i](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == j && std::get<1>(e) == i);}
														);
			auto activeDistTpl_T = std::find_if(activeDistList.begin(), activeDistList.end(),
														[j, i](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == i && std::get<1>(e) == j);}
														);
			auto passiveDistTpl_T = std::find_if(passiveDistList.begin(), passiveDistList.end(),
														[j, i](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == i && std::get<1>(e) == j);}
														);

			double dist = 0.0;
			int cnt = 0;

			if (activeDistTpl != activeDistList.end()) {
				dist += std::get<2>(*activeDistTpl);
				cnt += 1;
			}

			if (passiveDistTpl != passiveDistList.end() && std::get<2>(*passiveDistTpl) != -1) {
				dist += std::get<2>(*passiveDistTpl);
				cnt += 1;
			}

			if (activeDistTpl_T != activeDistList.end()) {
				dist += std::get<2>(*activeDistTpl);
				cnt += 1;
			}

			if (passiveDistTpl_T != passiveDistList.end() && std::get<2>(*passiveDistTpl) != -1) {
				dist += std::get<2>(*passiveDistTpl);
				cnt += 1;
			}

			if (cnt != 0) {
				dist /= cnt;
			}
			
			EDM[i][j] = dist;
			EDM[j][i] = dist;
		}
	}

	return EDM;
}

template <class T>
std::ostringstream
ConstructStringFromVector(std::vector<T> in_vectorData)
{
	std::ostringstream strVectorData;
	strVectorData << "[";
	for (const auto &element : in_vectorData) {
		strVectorData << element;
		if (&element != &in_vectorData.back()) {
			strVectorData << ",";
		}
	}
	strVectorData << "]";

	return strVectorData;
}

std::vector<std::tuple<double, double, double>>
GetEstimatedPositions(std::vector<std::vector<double>> in_EDM, PositionList in_anchorNodesPos, std::vector<int> in_anchorNodesIdx)
{
	std::vector<std::vector<double>> nodesPosMatrix = PositionListToVectorList(in_anchorNodesPos);
	
	std::ostringstream strAnchorNodesPos = ConstructStringMatrix<double>(nodesPosMatrix);
	std::ostringstream strEDM = ConstructStringMatrix<double>(in_EDM);
	std::ostringstream strNodesIdx = ConstructStringFromVector<int>(in_anchorNodesIdx);
	
	// sys.argv[2] is EDM, sys.argv[3] is anchor_nodes_pos, and sys.argv[4] is anchor_nodes_idx
	std::string command = "python3 graph.py --getLocs " + strEDM.str() + " " + strAnchorNodesPos.str() + " " + strNodesIdx.str();
	std::string output = exec(command);

	std::vector<std::vector<double>> vPositions;
	std::istringstream outputStream(output);
	std::string line;
	while (std::getline(outputStream, line)) {
		std::istringstream lineStream(line);
		std::vector<double> row;
		double element;
		while (lineStream >> element) {
			row.push_back(element);
		}
		vPositions.push_back(row);
	}

	std::vector<std::tuple<double, double, double>> estimatedPositions;

	for (const auto &row : vPositions) {
		assert (row.size() == 2);
		double x = row[0];
		double y = row[1];
		estimatedPositions.push_back({x, y, 0.0});
	}

	assert (estimatedPositions.size() == in_EDM.size());

	return estimatedPositions;
}

std::vector<std::tuple<double, double, double>>
CalculateP2Pposision (EnvConfig in_envConf, PositionList in_anchorNodesPos, std::vector<int> in_anchorIdx)
{
	std::vector<std::vector<double>> EDM;
	EDM = GetEDM(in_envConf);

	return GetEstimatedPositions(EDM, in_anchorNodesPos, in_anchorIdx);
}


Position
CalculatePosition(size_t in_staIdx, EnvConfig in_envConf, PositionList in_apPosList)
{
	double distArray[in_envConf.nAPs];
	for (size_t i=0; i<in_envConf.nAPs; i++) {
		auto distTpl = std::find_if(activeDistList.begin(), activeDistList.end(),
														[i, in_staIdx](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == i && std::get<1>(e) == in_staIdx);}
														);
		distArray[i] = std::get<2>(*distTpl);

		if (distArray[i] == -1) {
			return {-1000, -1000, 0};
		}
	}

	/* Calculate Distance */
	return LeastSquareError2D(in_apPosList, distArray);
}

double
CalculatePosDiff(Position in_staGroundTruthPos, Position in_staEstimatePos)
{
	double x_diff = std::get<0>(in_staGroundTruthPos) - std::get<0>(in_staEstimatePos);
	double y_diff = std::get<1>(in_staGroundTruthPos) - std::get<1>(in_staEstimatePos);
	double err_dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

	return err_dist;
}

std::list<double>
GetAvgDistDiff(int in_apIdx, int in_staIdx, Position in_apPositions, Position in_staGroundTruthPos)
{
	double x_dist = std::get<0>(in_apPositions) - std::get<0>(in_staGroundTruthPos);
	double y_dist = std::get<1>(in_apPositions) - std::get<1>(in_staGroundTruthPos);
	double dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

	auto distTpl = std::find_if(SessionRTTs.begin(), SessionRTTs.end(),
															[in_apIdx, in_staIdx](const std::tuple<size_t, size_t, std::list<int64_t>>& e)
															{return (std::get<0> (e) == in_apIdx && std::get<1>(e) == in_staIdx);}
															);

	std::list<int64_t> distList = std::get<2>(*distTpl);

	std::list<double> diffList;

	for (int64_t x : distList) {
		if (x*pow(10, -12)*299792458/2 > 1000 or x*pow(10, -12)*299792458/2 < 0) {
			continue;
		}
		double diff = (x*pow(10, -12)*299792458/2) - dist;
		diffList.push_back(diff);
		}

	return diffList;
}

double
GetThroughput(ApplicationContainer in_serverApp, uint32_t in_payloadSize, double in_simulationTime)
{
	double throughput = 0;
	uint64_t rxBytes = 0;

	for (uint32_t i=0; i < in_serverApp.GetN(); i++) {
		rxBytes += in_payloadSize * DynamicCast<UdpServer> (in_serverApp.Get(i))->GetReceived();
	}

	throughput = (rxBytes * 8) / ((in_simulationTime-1) * 1000000.0); // Mbit / s

	return throughput;
}

double
GetPacketLoss(ApplicationContainer in_serverApp, std::vector<ApplicationContainer> in_clientApps, uint32_t in_payloadSize)
{
	uint64_t rxPackets = 0, txPackets = 0;
	for (size_t i=0; i<in_serverApp.GetN(); i++) {
		rxPackets += DynamicCast<UdpServer> (in_serverApp.Get(i))->GetReceived();
	}

	size_t cleintSize = in_clientApps.size();
	for (size_t i=0; i<cleintSize; i++) {
		txPackets += DynamicCast<UdpClient> (in_clientApps[i].Get(0))->GetTotalTx() / in_payloadSize;
	}

	double packetLossRate = double (txPackets - rxPackets) / (double(txPackets)); // K Packets / s

	return packetLossRate;
}

void
RunSession(Ptr<FtmSession> in_session)
{
	in_session->SessionBegin();
}

std::tuple<double, double, double, double, double>
RunSimulation(uint32_t in_seed, uint8_t in_nBursts, EModel in_e, EnvConfig in_envConf, UdpConfig in_udpConfig, double in_simulationTime)
{
	// PositionList staPosList(glob_staPosList.begin(), glob_staPosList.begin()+in_envConf.nSTAs);

	double alpha = in_envConf.alpha;

	RngSeedManager::SetSeed(in_seed);
	RngSeedManager::SetRun(in_seed);

	Config::SetDefault("ns3::WifiMac::FTM_Enabled", BooleanValue(true));
	Config::SetDefault("ns3::WifiMac::CentralizedScheduler_Enabled", BooleanValue(true));

	WifiEnvironment wifiEnv = WifiEnvironment(in_envConf.nAPs,
																						in_envConf.nSTAs,
																						in_envConf.mcs,
																						in_udpConfig.payloadSize,
																						in_simulationTime,
																						in_udpConfig.udpInterval);
	Multilateration positioning = Multilateration(in_e, in_envConf.channelWidth);

	std::string strChannelSettings = "{0, " + std::to_string(in_envConf.channelWidth) + ", BAND_5GHZ, 0}";

	wifiEnv.CreateNodes();
	wifiEnv.SetupDevicePhy(in_seed, strChannelSettings);
	wifiEnv.SetupMobility();
	wifiEnv.ConstructDeviceLists();

	std::vector<std::vector<int>> independentSets = GetIndependentSets(wifiEnv.GetStaPositions());
	std::set<std::vector<int>> peerLinks = GetPeerLinksFromIndependentSets(independentSets);

	wifiEnv.SetupCentralizedScheduler(alpha,
																		MilliSeconds(1000/in_nBursts),
																		TransmissionType::FTM,
																		independentSets);
	
	wifiEnv.SetupApplication();

	WifiNetDevicesList wifiAPlist = wifiEnv.GetWifiAPs();
	WifiNetDevicesList wifiSTAlist = wifiEnv.GetWifiSTAs();
	AddressList apAddrList = wifiEnv.GetApAddress();
	AddressList staAddrList = wifiEnv.GetStaAddress();

	positioning.SetFTMParams(in_nBursts, in_simulationTime, in_envConf.nSTAs, alpha);
	positioning.ConstructActiveSessions(in_envConf, wifiSTAlist, staAddrList, peerLinks);
	positioning.ConstructPassiveSessions(in_envConf, wifiSTAlist, staAddrList);

	positioning.ConstructPeerDistance(in_envConf, wifiSTAlist, wifiEnv.GetStaPositions(), staAddrList);

	/*TODO: Missing the part where select 3 nodes to do Infrastructure Positioning */

	SessionList allSessions = positioning.GetAllSessions();

	for (Ptr<FtmSession> session : allSessions) {
		Simulator::Schedule(Seconds(1), &RunSession, session);
	}
	
	Simulator::Schedule (Seconds (0.0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);
	Simulator::Stop(Seconds (in_simulationTime));
	Simulator::Run();
	
	ApplicationContainer serverApp = wifiEnv.GetServerApps();
	std::vector<ApplicationContainer> clientAppList = wifiEnv.GetClientApps();

	positioning.EndAllSessions();

	PositionList staGroundTruthPosList = wifiEnv.GetStaPositions();
	PositionList apPosList = wifiEnv.GetApPositions();

	double appThroughput = 0.0;

	appThroughput += GetThroughput(serverApp, in_udpConfig.payloadSize, in_simulationTime);

	double packetLossRate = GetPacketLoss(serverApp, clientAppList, 1500);

	Simulator::Destroy();
		
	double avgAppThroughput = appThroughput / in_envConf.nSTAs;

	// std::cout << avgAppThroughput << "," << std::endl;

	double totalErr = 0.0;

	PositionList anchorSTAPoses(staGroundTruthPosList.begin(), staGroundTruthPosList.begin() + 3);
	std::vector<int> anchorIdx = {0, 1, 2};
	PositionList estimatedNodesPos = CalculateP2Pposision(in_envConf, anchorSTAPoses, anchorIdx);

	for (int i=0; i<in_envConf.nSTAs; i++) {
		totalErr += CalculatePosDiff(staGroundTruthPosList[i], estimatedNodesPos[i]);
	}

	// for (int i=0; i<in_envConf.nSTAs; i++) {
	// 	Position estimatedPos = CalculatePosition(i, in_envConf, apPosList);

	// 	if (std::get<0>(estimatedPos) == -1000 && std::get<1>(estimatedPos) == -1000) {
	// 		lossSTACount += 1;
	// 		continue;
	// 	}
	// 	totalErr += CalculatePosDiff(staGroundTruthPosList[i], estimatedPos);
	// }

	double ftmDiaglossRate = 0.0;
	int totalSessionsNum = DialogsCntList.size(); 
	for (int i=0; i<totalSessionsNum; i++) {
		ftmDiaglossRate += double(in_nBursts * 10 * in_simulationTime - DialogsCntList[i]) / double(in_nBursts * 10 * in_simulationTime);
	}

	return {
		appThroughput,
		avgAppThroughput,
		totalErr / in_envConf.nSTAs,
		packetLossRate,
		ftmDiaglossRate/totalSessionsNum,
	};
}

int
main(int argc, char *argv[])
{
	Time::SetResolution(Time::PS);

	// LogComponentEnable("YansWifiChannel", LOG_LEVEL_DEBUG);
	// LogComponentEnable("PhyEntity", LOG_LEVEL_ERROR);
	// LogComponentEnable("WifiPhyStateHelper", LOG_LEVEL_DEBUG);
	// LogComponentEnable("WifiMac", LOG_LEVEL_DEBUG);
	// LogComponentEnable("Txop", LOG_LEVEL_DEBUG);
	// LogComponentEnable("ChannelAccessManager", LOG_LEVEL_DEBUG);
	// LogComponentEnable("CentralizedScheduler", LOG_LEVEL_DEBUG);
	// LogComponentEnable("FrameExchangeManager", LOG_LEVEL_DEBUG);
	// LogComponentEnable("QosFrameExchangeManager", LOG_LEVEL_DEBUG);
	// LogComponentEnable("HtFrameExchangeManager", LOG_LEVEL_DEBUG);
	// LogComponentEnable("VhtFrameExchangeManager", LOG_LEVEL_DEBUG);
	// LogComponentEnable("FtmSession", LOG_LEVEL_ERROR);
	// LogComponentEnable("WifiMacQueue", LOG_LEVEL_DEBUG);

	const double simulationTime = 2.0;

	
	std::cout << "begin simulation" << std::endl;

	// size_t staPerAP = 10;
	int bps = 2;
  
	for (size_t staPerAP=10; staPerAP<20; staPerAP+=10) {
		std::cout << "# STA: " << staPerAP*3 << ", With CS " << std::endl;
		std::vector<std::tuple<double, double, double, double, double>> resultsList;
		
		EnvConfig envConf = {
			3, // nAPs
			3*staPerAP, // nSTAs
			4, // mcs
			40, // channelWidth
			0.3 // alpha
		};
		UdpConfig udpConf = {
			1500, // payloadSize
			"0.0012" // udpInterval
		};

		for (int simNum=1; simNum<11; simNum++) {
			std::cout << "Simulation: " << simNum << std::endl;
			resultsList.push_back(RunSimulation(simNum, bps, EModel::WIRED_ERROR, envConf, udpConf, simulationTime));
			
			std::cout << "PacketLossRate" << std::endl;
			for (auto &tupItr : resultsList) {
				std::cout << std::get<3>(tupItr) << "," << std::endl;
			}
			std::cout << "FtmDialogLossRate" << std::endl;
			for (auto &tupItr : resultsList) {
				std::cout << std::get<4>(tupItr) << "," << std::endl;
			}
		
			std::cout << "TotalThroughput" << std::endl;
			for (auto &tupItr : resultsList) {
				std::cout << std::get<0>(tupItr) << "," << std::endl;
			}
			std::cout << "AvgThroughput" << std::endl;
			for (auto &tupItr : resultsList) {
				std::cout << std::get<1>(tupItr) << "," << std::endl;
			}
			std::cout << "DistErr" << std::endl;
			for (auto &tupItr : resultsList) {
				std::cout << std::get<2>(tupItr) << "," << std::endl;
			}
		}

	}
	

	return 0;
}