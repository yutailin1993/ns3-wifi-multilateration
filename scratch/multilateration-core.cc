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
#include "ns3/ftm-header.h"
#include "ns3/ftm-session.h"
#include "ns3/mgt-headers.h"
#include "ns3/ftm-error-model.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"

#include "multilateration-core.h"

#include <tuple>
#include <vector>

void
SessionOver(FtmSession in_session)
{
	NS_LOG_UNCOND ((in_session.GetMeanRTT()*pow(10, -12)*299792458/2) << ",");
}

/* WifiEnvironment class implementation */
WifiEnvironment::~WifiEnvironment()
{
	m_apPositions.clear();
	m_APDevices.clear();
	m_STADevices.clear();
	m_wifiAPs.clear();
	m_wifiSTAs.clear();
	m_recvAddrs.clear();

	return;
}

void
WifiEnvironment::CreateNodes()
{
	m_wifiNodes.Create( m_nAPs + m_nSTAs );

	return;
}

void
WifiEnvironment::SetupDevicePhy(int64_t in_seed)
{
	m_wifi.SetStandard(standard);
	std::ostringstream oss;
	oss << "HeMcs" << m_mcs;

	m_yansWifiPhy.Set("RxGain", DoubleValue(0));
	m_yansWifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

	m_yansWifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	m_yansWifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));
	m_yansWifiPhy.SetChannel(m_yansWifiChannel.Create());

	m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
																 "DataMode", StringValue(oss.str()),
																 "ControlMode", StringValue("OfdmRate24Mbps"));

	m_wifiMac.SetType("ns3::AdhocWifiMac");

	m_devices = m_wifi.Install(m_yansWifiPhy, m_wifiMac, m_wifiNodes);

	int64_t streamNumber = 10*in_seed;
	streamNumber += m_wifi.AssignStreams(m_devices, streamNumber);
}

void
WifiEnvironment::SetRTSCTS(bool in_enableRTSCTS)
{
	m_enableRTSCTS = in_enableRTSCTS;
  UintegerValue ctsThr = (m_enableRTSCTS ? UintegerValue (10) : UintegerValue (22000));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
}

void
WifiEnvironment::SetupMobility(Position in_staPosition)
{
	m_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

	m_apPosAlloc = CreateObject<ListPositionAllocator> ();
	
	for (Position posIter : m_apPositions) {
		m_apPosAlloc->Add(Vector(std::get<0>(posIter),
														 std::get<1>(posIter),
														 std::get<2>(posIter)));
	}
	m_mobility.SetPositionAllocator(m_apPosAlloc);
	for (size_t i=0; i<m_nAPs; i++) {
		m_mobility.Install(m_wifiNodes.Get(i));
	}

	m_staPosAlloc = CreateObject<ListPositionAllocator> ();
	m_staPosAlloc->Add(Vector(std::get<0>(in_staPosition),
														std::get<1>(in_staPosition),
														std::get<2>(in_staPosition)));
	
	m_mobility.SetPositionAllocator(m_staPosAlloc);
	m_mobility.Install(m_wifiNodes.Get(m_nAPs + 0));
}

void
WifiEnvironment::SetupFTMEnv()
{
	// Set AP Devices, AP addresses, wifi AP devices
	for (int i=0; i<m_nAPs; i++) {
		m_APDevices.push_back(GetDevice(true, i));
		m_recvAddrs.push_back(m_APDevices[i]->GetAddress());
		m_wifiAPs.push_back(m_APDevices[i]->GetObject<WifiNetDevice> ());
	}
	// Add STA devices, wifi STA devices
	for (int i=0; i<m_nSTAs; i++) {
		m_STADevices.push_back(GetDevice(false, i));
		m_wifiSTAs.push_back(m_STADevices[i]->GetObject<WifiNetDevice> ());
	}

	m_yansWifiPhy.EnablePcap("multilateration", m_devices);
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

AddressList
WifiEnvironment::GetRecvAddress()
{
	return m_recvAddrs;
}

Ptr<NetDevice>
WifiEnvironment::GetDevice(bool in_getAPs, int in_deviceNo)
{
	if (in_getAPs) {
		return m_devices.Get(in_deviceNo);
	} else {
		return m_devices.Get(m_nAPs + in_deviceNo);
	}
}

/* Multilateration class implementation */
Multilateration::~Multilateration()
{
	m_ftmMap->~FtmMap();
	m_ftmParams.~FtmParams();
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
Multilateration::GenerateFTMSession(Ptr<WifiNetDevice> in_AP, Ptr<WifiNetDevice> in_STA, Address in_recvAddr)
{
	Ptr<WifiMac> staMac = in_STA->GetMac()->GetObject<WifiMac> ();
	Mac48Address toAddr = Mac48Address::ConvertFrom(in_recvAddr);
	Ptr<FtmSession> session = staMac->NewFtmSession(toAddr);

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


	session->SetFtmParams(m_ftmParams);
	session->SetSessionOverCallback(MakeCallback(&SessionOver));
	
	return session;
}

SessionList
Multilateration::GetAllSessions()
{
	return m_sessionList;
}

void
Multilateration::ConstructAllSessions(EnvConfig in_envConf, WifiNetDevicesList in_APs, WifiNetDevicesList in_STAs, AddressList in_recvAddrs)
{
	for (size_t i=0; i<in_envConf.nSTAs; i++) {
		for (size_t j=0; j<in_envConf.nAPs; j++) {
			Ptr<WifiNetDevice> sta =  in_STAs[i];
			Ptr<WifiNetDevice> ap = in_APs[j];
			Address recvAddr = in_recvAddrs[j];
			m_sessionList.push_back(GenerateFTMSession(ap, sta, recvAddr));
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
	ftmMap->LoadMap("./src/wifi/ftm_map/100x100.map");

	return ftmMap;
}

void
Multilateration::SetFTMParams(uint8_t in_nBursts)
{
	m_ftmParams.SetStatusIndication(FtmParams::RESERVED);
	m_ftmParams.SetStatusIndicationValue(0);
	m_ftmParams.SetNumberOfBurstsExponent(in_nBursts);
	m_ftmParams.SetBurstDuration(10);

	m_ftmParams.SetMinDeltaFtm(10);
	m_ftmParams.SetPartialTsfNoPref(true);
	m_ftmParams.SetAsap(true);
	m_ftmParams.SetFtmsPerBurst(2);
	m_ftmParams.SetBurstPeriod(1);
}

