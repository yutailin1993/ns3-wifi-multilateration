#include <cmath>

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

const PositionList glob_staPosList = {
	{3, 1, 0},
	{10, 7, 0},
	{16, 2, 0},
	{19, 8, 0},
	{7, 18, 0},
	{13, 8, 0},
	{2, 3, 0},
	{7, 12, 0},
	{1, 11, 0},
	{18, 2, 0},
	{7, 19, 0},
	{6, 8, 0},
	{10, 10, 0}
};

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

Position
CalculatePosition(size_t in_staIdx, EnvConfig in_envConf)
{
	double distArray[in_envConf.nAPs];
	for (size_t i=0; i<in_envConf.nAPs; i++) {
		auto distTpl = std::find_if(ApStaDistList.begin(), ApStaDistList.end(),
														[i, in_staIdx](const std::tuple<size_t, size_t, double>& e)
														{return (std::get<0>(e) == i && std::get<1>(e) == in_staIdx);}
														);
		distArray[i] = std::get<2>(*distTpl);
	}

	/* Calculate Distance */
	PositionList apPosList (APPositionCandidate.begin(), APPositionCandidate.begin()+in_envConf.nAPs);

	return LeastSquareError2D(apPosList, distArray);
}

double
GetThroughput(ApplicationContainer in_serverApp, uint32_t in_payloadSize, double in_simulationTime)
{
	double throughput = 0;
	uint64_t rxBytes = 0;

	for (uint32_t i=0; i < in_serverApp.GetN(); i++) {
		rxBytes += in_payloadSize * DynamicCast<UdpServer> (in_serverApp.Get(i))->GetReceived();
	}

	throughput = (rxBytes * 8) / (in_simulationTime * 1000000.0); // Mbit / s

	return throughput;
}

double
GetPacketLoss(ApplicationContainer in_serverApp, ApplicationContainer in_clientApps[], uint32_t in_payloadSize)
{
	uint64_t rxPackets = 0, txPackets = 0;
	for (size_t i=0; i<in_serverApp.GetN(); i++) {
		rxPackets += DynamicCast<UdpServer> (in_serverApp.Get(i))->GetReceived();
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

void
RunSimulation(uint32_t in_seed, uint8_t in_nBursts, EModel in_e, EnvConfig in_envConf, UdpConfig in_udpConfig, double in_simulationTime)
{
	PositionList staPosList(glob_staPosList.begin(), glob_staPosList.begin()+in_envConf.nSTAs);

	RngSeedManager::SetSeed(in_seed);
	RngSeedManager::SetRun(in_seed);

	Config::SetDefault("ns3::WifiMac::FTM_Enabled", BooleanValue(true));

	WifiEnvironment wifiEnv = WifiEnvironment(in_envConf.nAPs,
																						in_envConf.nSTAs,
																						in_envConf.mcs,
																						in_udpConfig.payloadSize,
																						in_simulationTime,
																						in_udpConfig.udpInterval);
	Multilateration positioning = Multilateration(in_e, in_envConf.channelWidth);


	wifiEnv.CreateNodes();
	wifiEnv.SetupDevicePhy(in_seed);
	wifiEnv.SetupMobility();
	wifiEnv.SetupFTMEnv();
	
	wifiEnv.SetupApplication();

	WifiNetDevicesList wifiAPlist = wifiEnv.GetWifiAPs();
	WifiNetDevicesList wifiSTAlist = wifiEnv.GetWifiSTAs();
	AddressList apAddrList = wifiEnv.GetRecvAddress();

	positioning.SetFTMParams(in_nBursts, in_simulationTime);
	positioning.ConstructAllSessions(in_envConf, wifiAPlist, wifiSTAlist, apAddrList);
	SessionList allSessions = positioning.GetAllSessions();

	for (Ptr<FtmSession> session : allSessions) {
		Simulator::ScheduleNow(&RunSession, session);
	}
	
	// Simulator::Schedule (Seconds (0.0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);
	Simulator::Stop(Seconds (in_simulationTime+1));
	Simulator::Run();
	
	ApplicationContainer serverApp = wifiEnv.GetServerApps();
	double appThroughput = 0.0;


	appThroughput += GetThroughput(serverApp, in_udpConfig.payloadSize, in_simulationTime);

	Simulator::Destroy();
		
	double avgAppThroughput = appThroughput / in_envConf.nSTAs;

	std::cout << avgAppThroughput << "," << std::endl;

	// for (int i=0; i<in_envConf.nSTAs; i++) {
	// 	Position staPos = CalculatePosition(i, in_envConf);
	// 	std::cout << "STA " << i << " Pos: {" << std::get<0>(staPos) << ", " << std::get<1>(staPos) <<  ", 0}" << std::endl;
	// }
}

int
main(int argc, char *argv[])
{
	Time::SetResolution(Time::PS);

	const double simulationTime = 9.0;
	
	std::cout << "begin simulation" << std::endl;

	for (size_t staPerAP=1; staPerAP<8; staPerAP++) {
		std::cout << "Station Num: " << staPerAP*3 << std::endl;
		
		EnvConfig envConf = {
			3, // nAPs
			3*staPerAP, // nSTAs
			4, // mcs
			40 // channelWidth
		};
		UdpConfig udpConf = {
			1500, // payloadSize
			"0.0012" // udpInterval
		};

		std::cout << "Burst Num: 64" << std::endl;

		for (int simNum=10; simNum<11; simNum++) {
			RunSimulation(simNum, 6, EModel::WIRED_ERROR, envConf, udpConf, simulationTime);
		}
	}
	

	return 0;
}