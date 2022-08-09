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

#include <vector>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FtmExample");

enum EModel {
	NO_ERROR,
	WIRED_ERROR,
	WIRELESS_ERROR
};

int channelWidth = 40;
int numSimulations = 20;
double distance = 100.0;
uint8_t nBurst = 4;

Ptr<WirelessFtmErrorModel::FtmMap> ftm_map = CreateObject<WirelessFtmErrorModel::FtmMap> ();

void SessionOver (FtmSession session)
{
	NS_LOG_UNCOND ((session.GetMeanRTT()*pow(10, -12)*299792458/2) - distance << ",");
}

static WiredFtmErrorModel::ChannelBandwidth GetWiredErrorChannelBandwidth(int channelWidth)
{
	switch (channelWidth)
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

static Ptr<WiredFtmErrorModel> GenerateWiredErrorModel()
{
	Ptr<WiredFtmErrorModel> error_model = CreateObject<WiredFtmErrorModel> ();
	error_model->SetChannelBandwidth(GetWiredErrorChannelBandwidth(channelWidth));

	return error_model;
}

static Ptr<WirelessFtmErrorModel> GenerateWirelessErrorModel(Ptr<WifiNetDevice> sta)
{
	Ptr<WirelessFtmErrorModel> error_model = CreateObject<WirelessFtmErrorModel> ();
	error_model->SetFtmMap(ftm_map);
	error_model->SetNode(sta->GetNode());
	error_model->SetChannelBandwidth(GetWiredErrorChannelBandwidth(channelWidth));

	return error_model;
}

static void GenerateTraffic (Ptr<WifiNetDevice> ap, Ptr<WifiNetDevice> sta, Address recvAddr, uint8_t nBurst, EModel e)
{
	Ptr<WifiMac> sta_mac = sta->GetMac()->GetObject<WifiMac>();
	Mac48Address to = Mac48Address::ConvertFrom(recvAddr);
	Ptr<FtmSession> session = sta_mac->NewFtmSession(to);
	
	if (session == 0) {
		NS_FATAL_ERROR ("FTM not enabled!");
	}

	switch (e)
	{
	case EModel::WIRED_ERROR:
		/* Deploy wired error model */
		session->SetFtmErrorModel(GenerateWiredErrorModel());
		break;

	case EModel::WIRELESS_ERROR:
		/* Deploy wireless error model */
		session->SetFtmErrorModel(GenerateWirelessErrorModel(sta));
		break;
	
	case EModel::NO_ERROR:
		/* Not deploy error model */
		break;

	default:
		NS_FATAL_ERROR ("Undefined Error model!");
	}

	//create the parameter for this session and set them
	FtmParams ftm_params;
	ftm_params.SetStatusIndication(FtmParams::RESERVED);
	ftm_params.SetStatusIndicationValue(0);
	ftm_params.SetNumberOfBurstsExponent(nBurst);
	ftm_params.SetBurstDuration(10);

	ftm_params.SetMinDeltaFtm(10);
	ftm_params.SetPartialTsfNoPref(true);
	ftm_params.SetAsap(true);
	ftm_params.SetFtmsPerBurst(2);
	ftm_params.SetBurstPeriod(1);

	session->SetFtmParams(ftm_params);
	session->SetSessionOverCallback(MakeCallback(&SessionOver));
	session->SessionBegin();
}

void RunSimulation(uint32_t seed, uint8_t nBurst, EModel e)
{
	double rss = -80;
	RngSeedManager::SetSeed(seed);
	RngSeedManager::SetRun(seed);

	Config::SetDefault("ns3::WifiMac::FTM_Enabled", BooleanValue(true));

	NodeContainer c;
	c.Create(2);


	WifiHelper wifi;
	wifi.SetStandard(WIFI_STANDARD_80211n);

	YansWifiPhyHelper wifiPhy;
	wifiPhy.Set("RxGain", DoubleValue(0));
	// ns3 supports RadioTap and Prism tracing extensions for 802.11b
	wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss("ns3::FixedRssLossModel", "Rss", DoubleValue(rss));
	wifiPhy.SetChannel(wifiChannel.Create());

	WifiMacHelper wifiMac;
	wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
															 "DataMode", StringValue("HtMcs4"),
															 "ControlMode", StringValue("OfdmRate24Mbps"));

	Ssid ssid = Ssid("wifi-default");
	wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
	
	NetDeviceContainer staDevice = wifi.Install(wifiPhy, wifiMac, c.Get(0));
	NetDeviceContainer devices = staDevice;

	wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
	NetDeviceContainer apDevice = wifi.Install(wifiPhy, wifiMac, c.Get(1));
	devices.Add(apDevice);

	int64_t streamNumber = 10*seed;
	streamNumber += wifi.AssignStreams(apDevice, streamNumber);
	streamNumber += wifi.AssignStreams(staDevice, streamNumber);


	// Mobility Model
	MobilityHelper mobility;

	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add (Vector(distance, 0, 0));
	mobility.SetPositionAllocator (positionAlloc);
	mobility.Install(c.Get(0));

	Ptr<ListPositionAllocator> positionAlloc2 = CreateObject<ListPositionAllocator>();
	positionAlloc2->Add (Vector(0.0, 0.0, 0.0));
	mobility.SetPositionAllocator(positionAlloc2);
	mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mobility.Install(c.Get(1));

	Ptr<NetDevice> ap = apDevice.Get(0);
	Ptr<NetDevice> sta = staDevice.Get(0);
	Address recvAddr = ap->GetAddress();

	Ptr<WifiNetDevice> wifi_ap = ap->GetObject<WifiNetDevice>();
	Ptr<WifiNetDevice> wifi_sta = sta->GetObject<WifiNetDevice>();

	wifiPhy.EnablePcap("ftm-example", devices);
	Simulator::ScheduleNow(&GenerateTraffic, wifi_ap, wifi_sta, recvAddr, nBurst, e);
	

	Simulator::Stop(Seconds (10.0));
	Simulator::Run();
	Simulator::Destroy();
}

int main (int argc, char *argv[])
{
	// ftm_map->LoadMap("./src/wifi/ftm_map/100x100.map");
	Time::SetResolution(Time::PS);

	// int nBurst_list[5] = {1, 2, 4, 5, 6};
	int dist_list[7] = {8, 16, 20, 30, 40, 50, 100};
	int bw_list[2] = {20, 40};

	for (int i=0; i<2; i++) {
		for (int j=0; j<7; j++) {
			channelWidth = bw_list[i];
			distance = dist_list[j];
			std::cout << "Channel width: " << channelWidth << ", No. Burst: " << pow(2, nBurst) << ", Distance: " << distance << std::endl;
	
			std::cout << "Wired Error Model" << std::endl;
			for (int i=0; i<numSimulations; i++) {
				RunSimulation(i+1, nBurst, EModel::WIRED_ERROR);
			}
		}
	}

	

	return 0;
}
