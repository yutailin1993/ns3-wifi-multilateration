#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/wifi-mac.h"
#include "ns3/ftm-header.h"
#include "ns3/ftm-session.h"
#include "ns3/mgt-headers.h"
#include "ns3/ftm-error-model.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/centralized-scheduler.h"

#include "multilateration-core.h"

#include <tuple>
#include <vector>
#include <set>
#include <tgmath.h>

WifiStandard standard = WIFI_STANDARD_80211ax;
double rss = -70;

void
SessionOver(FtmSession in_session)
{
	std::tuple<size_t, size_t> connectionPair = in_session.GetSessionBelonging();
	size_t sta1_Idx = std::get<0>(connectionPair);
	size_t sta2_Idx = std::get<1>(connectionPair);
	double distance = 0;

	if (sta1_Idx == -1) { // skip broadcast sessions
		return;
	}
	
	if (in_session.GetMeanRTT() == -1) {
		distance = -1;
	} else {
		distance = in_session.GetMeanRTT()*pow(10, -12)*299792458;
	}

	activeDistList.push_back({sta1_Idx, sta2_Idx, distance});

	// std::cout << "AP: " << apIdx << ", STA: " << staIdx << ", Successful FTM Count: " << in_session.GetIndividualRTT().size() << ", Distance: " << distance << std::endl;
	std::map<uint8_t, Ptr<FtmSession::FtmDialog>> dialogs = in_session.GetFtmDialogs();

	std::list<int64_t> theList = in_session.GetIndividualRTT();

	SessionRTTs.push_back({sta1_Idx, sta2_Idx, theList});
	DialogsCntList.push_back(dialogs.size());
}

void
PassiveSessionOver(FtmSession in_session)
{
	std::tuple<size_t, size_t> connectionPair = in_session.GetSessionBelonging();
	size_t sta1_Idx = std::get<0>(connectionPair);
	size_t sta2_Idx = std::get<1>(connectionPair);
	double distance = 0;

	in_session.PassiveCalculateRTT();
	
	if (in_session.GetMeanRTT() == -1) {
		distance = -1;
	} else {
		distance = in_session.GetMeanRTT()*pow(10, -12)*299792458/2;
	}

	passiveDistList.push_back({sta1_Idx, sta2_Idx, distance});

	// std::cout << "AP: " << apIdx << ", STA: " << staIdx << ", Successful FTM Count: " << in_session.GetIndividualRTT().size() << ", Distance: " << distance << std::endl;
	std::map<uint8_t, Ptr<FtmSession::FtmDialog>> dialogs = in_session.GetFtmDialogs();

	std::list<int64_t> theList = in_session.GetIndividualRTT();
	
	SessionRTTs.push_back({sta1_Idx, sta2_Idx, theList});
	DialogsCntList.push_back(dialogs.size());
}

/* WifiEnvironment class implementation */
WifiEnvironment::~WifiEnvironment()
{
	m_apPositions.clear();
	m_APDevicesList.clear();
	m_STADevicesList.clear();
	m_wifiAPs.clear();
	m_wifiSTAs.clear();
	m_apAddrs.clear();

	return;
}

void
WifiEnvironment::CreateNodes()
{
	m_wifiApNodes.Create(m_nAPs);

	for (auto &staNodes : m_wifiStaNodeGroups) {
		staNodes.Create(m_nSTAs / 3);
		m_wifiStaNodes.Add(staNodes);
	}

	m_wifiNodes.Add(m_wifiApNodes);
	m_wifiNodes.Add(m_wifiStaNodes);
}

void
WifiEnvironment::SetupDevicePhy(int64_t in_seed, std::string in_strChannelSettings)
{
	m_wifi.SetStandard(standard);
	std::ostringstream oss;
	oss << "HeMcs" << m_mcs;

	// m_yansWifiPhy.Set("RxGain", DoubleValue(0));
	m_yansWifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

	m_yansWifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	// m_yansWifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));
	m_yansWifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));

	m_yansWifiPhy.SetChannel(m_yansWifiChannel.Create());

	m_yansWifiPhy.Set("ChannelSettings", StringValue (in_strChannelSettings));

	m_wifi.SetRemoteStationManager("ns3::IdealWifiManager");
	// m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
	// 															 "DataMode", StringValue(oss.str()),
	// 															 "ControlMode", StringValue("OfdmRate24Mbps"));

	m_wifiMac.SetType("ns3::AdhocWifiMac");

	for (size_t i=0; i<3; i++) {
		m_staDeviceGroups[i] = m_wifi.Install(m_yansWifiPhy, m_wifiMac, m_wifiStaNodeGroups[i]);
		m_staDevices.Add(m_staDeviceGroups[i]);
	}
	m_apDevices = m_wifi.Install(m_yansWifiPhy, m_wifiMac, m_wifiApNodes);
	m_devices.Add(m_staDevices);
	m_devices.Add(m_apDevices);

	int64_t streamNumber = 10*in_seed;
	streamNumber += m_wifi.AssignStreams(m_staDevices, streamNumber);
	streamNumber += m_wifi.AssignStreams(m_apDevices, streamNumber);
}

void
WifiEnvironment::SetRTSCTS(bool in_enableRTSCTS)
{
	m_enableRTSCTS = in_enableRTSCTS;
  UintegerValue ctsThr = (m_enableRTSCTS ? UintegerValue (10) : UintegerValue (22000));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}

NodeContainer
WifiEnvironment::GetWifiNodes()
{
	return m_wifiNodes;
}

void
WifiEnvironment::SetupMobility()
{
	m_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	m_apPosAlloc = CreateObject<ListPositionAllocator> ();
	
	for (Position posIter : m_apPositions) {
		m_apPosAlloc->Add(Vector(std::get<0>(posIter),
														 std::get<1>(posIter),
														 std::get<2>(posIter)));
	}
	m_mobility.SetPositionAllocator(m_apPosAlloc);
	m_mobility.Install(m_wifiApNodes);

	m_staPosAlloc = 
		CreateObjectWithAttributes<RandomDiscPositionAllocator> ("X", StringValue("0"),
																														 "Y", StringValue("0"),
																														 "Rho", StringValue("ns3::UniformRandomVariable[Min=0|Max=70]"));
	m_mobility.SetPositionAllocator(m_staPosAlloc);
	m_mobility.Install(m_wifiStaNodes);
	
	for (size_t i=0; i<m_nAPs; i++) {
		Ptr<MobilityModel> mob = m_wifiApNodes.Get(i)->GetObject<MobilityModel>();
		m_apPositions.push_back({mob->GetPosition().x, mob->GetPosition().y, mob->GetPosition().z});
	}

	for (size_t i=0; i<m_nSTAs; i++) {
		Ptr<MobilityModel> mob = m_wifiStaNodes.Get(i)->GetObject<MobilityModel>();
		m_staPositions.push_back({mob->GetPosition().x, mob->GetPosition().y, mob->GetPosition().z});
	}
}

void
WifiEnvironment::ConstructDeviceLists()
{
	// Set AP Devices, AP addresses, wifi AP devices
	for (int i=0; i<m_nAPs; i++) {
		m_APDevicesList.push_back(GetDevice(DeviceType::AP, i));
		m_apAddrs.push_back(m_APDevicesList[i]->GetAddress());
		m_wifiAPs.push_back(m_APDevicesList[i]->GetObject<WifiNetDevice> ());
	}
	// Add STA devices, wifi STA devices
	for (int i=0; i<m_nSTAs; i++) {
		m_STADevicesList.push_back(GetDevice(DeviceType::STA, i));
		m_staAddrs.push_back(m_STADevicesList[i]->GetAddress());
		m_wifiSTAs.push_back(m_STADevicesList[i]->GetObject<WifiNetDevice> ());
	}

	for (int i=0; i<m_nAPs+m_nSTAs; i++) {
		m_AllDevicesList.push_back(GetDevice(DeviceType::ALL, i));
		m_wifiAll.push_back(m_AllDevicesList[i]->GetObject<WifiNetDevice> ());
	}

	// m_yansWifiPhy.EnablePcap("multilateration", m_devices);
}

void
WifiEnvironment::SetupCentralizedScheduler(double in_alpha,
																					 Time in_periodLength,
																					 TransmissionType in_transType,
																					 std::vector<std::vector<int>> in_independentSets)
{

	m_centralizedScheduler = CreateObject<CentralizedScheduler> (in_alpha,
																															 in_periodLength, 
																															 in_transType, 
																															 in_independentSets);
	
	m_centralizedScheduler->ConstructStaAddrTable(m_staAddrs);

	for (int i=0; i<m_nSTAs+m_nAPs; i++) {
		m_wifiAll[i]->GetMac()->SetupCentralizedScheduler(i, m_centralizedScheduler);
	}
}

void
WifiEnvironment::SetupApplication()
{
	m_stack.Install(m_wifiNodes);

	m_ipv4Address.SetBase("192.168.1.0", "255.255.255.0");

	for (size_t i=0; i<3; i++) {
		m_staNodeGroupInterfaces[i] = m_ipv4Address.Assign(m_staDeviceGroups[i]);
	}
	m_apNodeInterfaces = m_ipv4Address.Assign(m_apDevices);

	UdpServerHelper server(m_udpPort);
	m_serverApp = server.Install(m_wifiStaNodes);
	m_serverApp.Start(Seconds (0.0));
	m_serverApp.Stop(Seconds (m_simulationTime));

	for (size_t i=0; i<3; i++) {
		uint32_t groupSize = m_wifiStaNodeGroups[i].GetN();
		for (size_t j=0; j<groupSize; j++) {
			UdpClientHelper client(m_staNodeGroupInterfaces[i].GetAddress(j), m_udpPort);
			client.SetAttribute("MaxPackets", UintegerValue(4294967295u));
			client.SetAttribute("Interval", TimeValue(Time(m_udpInterval)));
			client.SetAttribute("PacketSize", UintegerValue(m_payloadSize));

			ApplicationContainer clientApp = client.Install(m_wifiApNodes.Get(i));
			clientApp.Start(Seconds(1.1));
			clientApp.Stop(Seconds(m_simulationTime));

			m_clientApps.push_back(clientApp);
		}
	}
}

ApplicationContainer
WifiEnvironment::GetServerApps()
{
	return m_serverApp;
}

std::vector<ApplicationContainer>
WifiEnvironment::GetClientApps()
{
	return m_clientApps;
}

std::vector<Ptr<WifiNetDevice>>
WifiEnvironment::GetWifiAPs()
{
	return m_wifiAPs;
}

std::vector<Ptr<WifiNetDevice>>
WifiEnvironment::GetWifiSTAs()
{
	return m_wifiSTAs;
}

PositionList
WifiEnvironment::GetStaPositions()
{
	return m_staPositions;
}

PositionList
WifiEnvironment::GetApPositions()
{
	return m_apPositions;
}

AddressList
WifiEnvironment::GetApAddress()
{
	return m_apAddrs;
}

AddressList
WifiEnvironment::GetStaAddress()
{
	return m_staAddrs;
}

Ptr<NetDevice>
WifiEnvironment::GetDevice(DeviceType in_deviceType, int in_deviceNo)
{
	switch (in_deviceType)
	{
	case DeviceType::AP:
		return m_apDevices.Get(in_deviceNo);

	case DeviceType::STA:
		return m_staDevices.Get(in_deviceNo);

	case DeviceType::ALL:
		return m_devices.Get(in_deviceNo);
	
	default:
		NS_FATAL_ERROR("Unrecognized deviced type!");
	}
}

/* Multilateration class implementation */
Multilateration::~Multilateration()
{
	m_ftmParams.~FtmParams();
	activeDistList.clear();
	passiveDistList.clear();
	DialogsCntList.clear();
}

Ptr<WiredFtmErrorModel>
Multilateration::GenerateWiredErrorModel()
{
	Ptr<WiredFtmErrorModel> errorModel = CreateObject<WiredFtmErrorModel> ();
	errorModel->SetChannelBandwidth(Multilateration::GetErrorModel());

	return errorModel;
}	

Ptr<WirelessFtmErrorModel>
Multilateration::GenerateWirelessErrorModel(Ptr<WifiNetDevice> in_sta)
{
	Ptr<WirelessFtmErrorModel> errorModel = CreateObject<WirelessFtmErrorModel> ();
	errorModel->SetFtmMap(m_ftmMap);
	errorModel->SetNode(in_sta->GetNode());
	errorModel->SetChannelBandwidth(Multilateration::GetErrorModel());

	return errorModel;
}

Ptr<FtmSession>
Multilateration::GenerateDefaultFTMSession(Ptr<WifiNetDevice> in_STA, Address in_recvAddr, bool is_passive)
{
	Ptr<WifiMac> staMac = in_STA->GetMac()->GetObject<WifiMac> ();
	Mac48Address toAddr = Mac48Address::ConvertFrom(in_recvAddr);
	Ptr<FtmSession> session = staMac->NewFtmSession(toAddr, is_passive);

	if (session == 0) {
		NS_FATAL_ERROR("FTM not enabled!");
	}

	switch (m_errorModel) {
		case EModel::WIRED_ERROR:
			session->SetFtmErrorModel(GenerateWiredErrorModel());
			break;
		case EModel::WIRELESS_ERROR:
			session->SetFtmErrorModel(GenerateWirelessErrorModel(in_STA));
			break;
		case EModel::NO_ERROR:
			break;

		default:
			NS_FATAL_ERROR ("Undefined Error Model!");
	}

	return session;
}

Ptr<FtmSession>
Multilateration::GeneratePassiveFTMSession(std::tuple<size_t, size_t> in_connectionPair, Ptr<WifiNetDevice> in_STA, Address in_recvAddr)
{
	Ptr<FtmSession> session = GenerateDefaultFTMSession(in_STA, in_recvAddr, true);

	session->SetSessionBelonging(in_connectionPair);
	session->SetSessionOverCallback(MakeCallback(&PassiveSessionOver));

	return session;
}

Ptr<FtmSession>
Multilateration::GenerateFTMSession(std::tuple<size_t, size_t> in_connectionPair, Ptr<WifiNetDevice> in_STA, Address in_recvAddr)
{
	Ptr<FtmSession> session = GenerateDefaultFTMSession(in_STA, in_recvAddr, false);

	session->SetSessionBelonging(in_connectionPair);
	session->SetFtmParams(m_ftmParams);
	session->SetSessionOverCallback(MakeCallback(&SessionOver));
	
	return session;
}

double 
Multilateration::DistanceCalculate(double x1, double y1, double x2, double y2)
{
	double x = x1 - x2; //calculating number to square in next step
	double y = y1 - y2;
	double dist;

	dist = pow(x, 2) + pow(y, 2);       //calculating Euclidean distance
	dist = sqrt(dist);                  

	return dist;
}

std::map<int, double>
Multilateration::ComputeDistance(size_t in_nodeIdx, int in_nSTAs, PositionList in_posList)
{
	std::map<int, double> peerDistances;
	Position selfPos = in_posList[in_nodeIdx];
	for (int i=0; i<in_nSTAs; i++) {
		if (i==in_nodeIdx) {
			continue;
		}
		Position peerPos = in_posList[i];
		peerDistances.insert({i, DistanceCalculate(std::get<0>(selfPos), std::get<1>(selfPos),
																							 std::get<0>(peerPos), std::get<1>(peerPos))});
	}

	return peerDistances;
}

void
Multilateration::ConstructPeerDistance(EnvConfig in_envConf, WifiNetDevicesList in_STAs, PositionList in_posList, AddressList in_staAddrs)
{
	for (size_t i=0; i<in_envConf.nSTAs; i++) {
		Ptr<WifiNetDevice> sta = in_STAs[i];
		std::map<int, double> distanceList = ComputeDistance(i, in_envConf.nSTAs, in_posList);
		sta->GetMac()->SetPeerDistanceList(distanceList, in_staAddrs);
	}
}

SessionList
Multilateration::GetAllSessions()
{
	return m_sessionList;
}

void
Multilateration::ConstructIfSessions(std::vector<int> in_anchorSTAs, WifiNetDevicesList in_APs, WifiNetDevicesList in_STAs, AddressList in_apAddrs)
{
	for (auto &s : in_anchorSTAs) {
		for (size_t j=0; j<3; j++) {
			Ptr<WifiNetDevice> sta = in_STAs[s];
			Ptr<WifiNetDevice> ap = in_APs[j];
			Address recvAddr = in_apAddrs[j];
			std::tuple<size_t, size_t> connectionPair = {j, s};
			m_sessionList.push_back(GenerateFTMSession(connectionPair, sta, recvAddr));
		}
	}
}

SessionList
Multilateration::GetBroadcastSessions ()
{
	return m_broadcastSessionList;
}

void
Multilateration::ConstructActiveSessions(EnvConfig in_envConf, WifiNetDevicesList in_STAs, AddressList in_recvAddrs)
{
	for (size_t i=0; i<in_envConf.nSTAs; i++) {
		Ptr<WifiNetDevice> sta = in_STAs[i];
		for (size_t j=0; j<in_envConf.nSTAs; j++) {
			if (j == i) {
				continue;
			}
			Address recvAddr = in_recvAddrs[j];
			std::tuple<size_t, size_t> connectionPair = {j, i};
			m_sessionList.push_back(GenerateFTMSession(connectionPair, sta, recvAddr));
		}
		// broadcast session
		std::tuple<size_t, size_t> connectionPair = {-1, i};
		m_broadcastSessionList.push_back(GenerateFTMSession(connectionPair, sta, Mac48Address::GetBroadcast()));
	}
}

void
Multilateration::ConstructPassiveSessions(EnvConfig in_envConf, WifiNetDevicesList in_STAs, AddressList in_recvAddrs)
{
	for (size_t i=0; i<in_envConf.nSTAs; i++) {
		for (size_t j=0; j<in_envConf.nSTAs; j++) {
			if (j == i) {
				continue;
			}
			Ptr<WifiNetDevice> sta = in_STAs[i];
			Address recvAddr = in_recvAddrs[j];
			std::tuple<size_t, size_t> connectionPair = {j, i};
			m_passiveSessionList.push_back(GeneratePassiveFTMSession(connectionPair, sta, recvAddr));
		}
	}
}

WiredFtmErrorModel::ChannelBandwidth
Multilateration::GetErrorModel()
{
	switch (m_channelWidth)
	{
		case 20:
			return WiredFtmErrorModel::Channel_20_MHz;
		case 40:
			return WiredFtmErrorModel::Channel_40_MHz;
		case 80:
			return WiredFtmErrorModel::Channel_80_MHz;
		case 160:
			return WiredFtmErrorModel::Channel_160_MHz;
		default:
			NS_FATAL_ERROR ("Not supported channel!");
	}
}

Ptr<WirelessFtmErrorModel::FtmMap>
Multilateration::LoadWirelessErrorMap()
{
	Ptr<WirelessFtmErrorModel::FtmMap> ftmMap = CreateObject<WirelessFtmErrorModel::FtmMap> ();
	ftmMap->LoadMap("./src/wifi/ftm_map/50x50.map");

	return ftmMap;
}

void
Multilateration::SetFTMParams(int in_nBurstsPerSecond, double in_simulationTime, int in_nSTAs, double in_alpha)
{
	const uint16_t numOfMiniSec = 10; // 1 sec
	int totalBursts = int (in_nBurstsPerSecond*ceil(in_simulationTime-1));
	// int totalBursts = 54;
	int burstExponent = int (log2(totalBursts));

	int ftmPerBurst = std::min(5, std::max(1, int (floor(500 * in_alpha / (0.4 * in_nSTAs))) - 1));

	std::cout << "FTM per burst: " << ftmPerBurst << std::endl;

	m_ftmParams.SetStatusIndication(FtmParams::RESERVED);
	m_ftmParams.SetStatusIndicationValue(0);
	m_ftmParams.SetNumberOfBurstsExponent(burstExponent);
	m_ftmParams.SetBurstDuration(11);

	m_ftmParams.SetMinDeltaFtm(10);
	m_ftmParams.SetPartialTsfNoPref(true);
	m_ftmParams.SetAsap(true);
	m_ftmParams.SetFtmsPerBurst(ftmPerBurst);
	m_ftmParams.SetBurstPeriod((numOfMiniSec/in_nBurstsPerSecond));
}

void
Multilateration::EndAllSessions()
{
	for (auto &sessionIter : m_sessionList) {
		if (!sessionIter->GetSessionEnded()) {
			sessionIter->CallEndSession();
		}
	}

	for (auto &sessionIter : m_passiveSessionList) {
		if (!sessionIter->GetSessionEnded()) {
			sessionIter->CallEndSession();
		}
	}
}