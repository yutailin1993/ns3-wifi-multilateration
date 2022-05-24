#include "ns3/gnuplot.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/yans-wifi-channel.h"

using namespace ns3;

std::string tcpVariant = std::string("ns3::") + "TcpNewReno";

uint32_t payloadSize = 1472; // bytes
double simulationTime = 5; //seconds
double frequency = 5.0; // 5.0 GHz
double step = 10; //meters
bool shortGuardInterval = false;
bool preambleDetection = false;

double GetThroughput (ApplicationContainer serverApp)
{
	uint64_t totalPacketsThrough = DynamicCast<PacketSink> (serverApp.Get (0))->GetTotalRx ();
  double throughput = totalPacketsThrough * 8 / (simulationTime * 1000000.0); //Mbit/s

	return throughput;
}

void RunSimulation(uint8_t nStreams, Gnuplot &plot, std::string mode)
{
	std::string strTestSet = mode + "-" + std::to_string(nStreams) + "x" + std::to_string(nStreams);
	Gnuplot2dDataset dataset (strTestSet);
	std::cout << strTestSet << std::endl;
	for (double d=0; d<100; d+=step) {
		NodeContainer wifiStaNode;
		wifiStaNode.Create(1);
		NodeContainer wifiApNode;
		wifiApNode.Create(1);

		YansWifiChannelHelper channel;
		channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
		channel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));

		YansWifiPhyHelper wifiPhy;
		wifiPhy.SetChannel(channel.Create());
		wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");

		if (!preambleDetection) {
			wifiPhy.DisablePreambleDetectionModel();
		}

		wifiPhy.Set("Antennas", UintegerValue(nStreams));
		wifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(nStreams));
		wifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(nStreams));
		wifiPhy.Set("ChannelSettings", StringValue ("{0, 80, BAND_5GHZ, 0}"));

		WifiMacHelper mac;
		WifiHelper wifi;
		wifi.SetStandard(WIFI_STANDARD_80211ax);
		wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
																 "DataMode", StringValue(mode),
																 "ControlMode", StringValue(mode));
		Ssid ssid = Ssid("network");

		mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
		NetDeviceContainer apDevice;
		apDevice = wifi.Install(wifiPhy, mac, wifiApNode);

		mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
		NetDeviceContainer staDevice;
		staDevice = wifi.Install(wifiPhy, mac, wifiStaNode);

		// Set guard interval
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported", BooleanValue (shortGuardInterval));

		MobilityHelper mobility;
		Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

		positionAlloc->Add (Vector (0.0, 0.0, 0.0));
		positionAlloc->Add (Vector (d, 0.0, 0.0));
		mobility.SetPositionAllocator(positionAlloc);

		mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

		mobility.Install(wifiApNode);
		mobility.Install(wifiStaNode);

		InternetStackHelper stack;
		stack.Install(wifiApNode);
		stack.Install(wifiStaNode);

		Ipv4AddressHelper address;
		address.SetBase("192.168.0.0", "255.255.255.0");
		Ipv4InterfaceContainer staNodeInterface = address.Assign(staDevice);
		Ipv4InterfaceContainer apNodeInterface = address.Assign(apDevice);

		ApplicationContainer serverApp;

		uint16_t port = 50000;
    Address localAddress (InetSocketAddress (Ipv4Address::GetAny (), port));
		PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", localAddress);
		serverApp = sinkHelper.Install(wifiStaNode.Get(0));
		serverApp.Start(Seconds(0.0));
		serverApp.Stop(Seconds(simulationTime + 1));

		OnOffHelper onoff("ns3::TcpSocketFactory", Ipv4Address::GetAny());

		onoff.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
		onoff.SetAttribute ("PacketSize", UintegerValue(payloadSize));
		onoff.SetAttribute ("DataRate", DataRateValue(DataRate("1000Mbps")));
		AddressValue remoteAddress (InetSocketAddress (staNodeInterface.GetAddress(0), port));
		onoff.SetAttribute ("Remote", remoteAddress);
		ApplicationContainer clientApp = onoff.Install(wifiApNode.Get(0));
		clientApp.Start (Seconds (1.0));
		clientApp.Stop (Seconds (simulationTime + 1));

		Ipv4GlobalRoutingHelper::PopulateRoutingTables();

		Simulator::Stop (Seconds (simulationTime+1));
		Simulator::Run ();

		double throughput = GetThroughput(serverApp);

		dataset.Add(d, throughput);

		std::cout << "Distance: " << d << ", Throughput: " << throughput << " Mbps" << std::endl;

		Simulator::Destroy();

	}

	plot.AddDataset (dataset);

}

int main(int argc, char *argv[])
{
	std::vector <std::string> modes;
	modes.push_back("HeMcs0");
	modes.push_back("HeMcs1");
	modes.push_back("HeMcs2");
	modes.push_back("HeMcs3");
	modes.push_back("HeMcs4");
	modes.push_back("HeMcs5");
	modes.push_back("HeMcs6");
	modes.push_back("HeMcs7");
	modes.push_back("HeMcs8");
	modes.push_back("HeMcs9");
	modes.push_back("HeMcs10");
	modes.push_back("HeMcs11");

	std::ofstream file ("80211ax-mimo-throughput.plt");

	Gnuplot plot = Gnuplot ("80211ax-mimo-throughput.eps");

	// TCP global setting
	TypeId tcpTid;
  NS_ABORT_MSG_UNLESS (TypeId::LookupByNameFailSafe (tcpVariant, &tcpTid), "TypeId " << tcpVariant << " not found");
  Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (tcpVariant)));    NS_ABORT_MSG_UNLESS (TypeId::LookupByNameFailSafe (tcpVariant, &tcpTid), "TypeId " << tcpVariant << " not found");
	Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));

	for (uint32_t i = 0; i <modes.size(); i++) {
		auto strMode = modes[i];
		for (uint8_t nStreams=1; nStreams<5; nStreams++) {
			RunSimulation(nStreams, plot, strMode);
		}
	}

	plot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  plot.SetLegend ("Distance (Meters)", "Throughput (Mbit/s)");
  plot.SetExtra  ("set xrange [0:100]\n\
set yrange [0:800]\n\
set ytics 0,50,800\n\
set style line 1 dashtype 1 linewidth 5\n\
set style line 2 dashtype 1 linewidth 5\n\
set style line 3 dashtype 1 linewidth 5\n\
set style line 4 dashtype 1 linewidth 5\n\
set style line 5 dashtype 1 linewidth 5\n\
set style line 6 dashtype 1 linewidth 5\n\
set style line 7 dashtype 1 linewidth 5\n\
set style line 8 dashtype 1 linewidth 5\n\
set style line 9 dashtype 1 linewidth 5\n\
set style line 10 dashtype 1 linewidth 5\n\
set style line 11 dashtype 1 linewidth 5\n\
set style line 12 dashtype 1 linewidth 5\n\
set style line 13 dashtype 2 linewidth 5\n\
set style line 14 dashtype 2 linewidth 5\n\
set style line 15 dashtype 2 linewidth 5\n\
set style line 16 dashtype 2 linewidth 5\n\
set style line 17 dashtype 2 linewidth 5\n\
set style line 18 dashtype 2 linewidth 5\n\
set style line 19 dashtype 2 linewidth 5\n\
set style line 20 dashtype 2 linewidth 5\n\
set style line 21 dashtype 2 linewidth 5\n\
set style line 22 dashtype 2 linewidth 5\n\
set style line 23 dashtype 2 linewidth 5\n\
set style line 24 dashtype 2 linewidth 5\n\
set style line 25 dashtype 3 linewidth 5\n\
set style line 26 dashtype 3 linewidth 5\n\
set style line 27 dashtype 3 linewidth 5\n\
set style line 28 dashtype 3 linewidth 5\n\
set style line 29 dashtype 3 linewidth 5\n\
set style line 30 dashtype 3 linewidth 5\n\
set style line 31 dashtype 3 linewidth 5\n\
set style line 32 dashtype 3 linewidth 5\n\
set style line 33 dashtype 3 linewidth 5\n\
set style line 34 dashtype 3 linewidth 5\n\
set style line 35 dashtype 3 linewidth 5\n\
set style line 36 dashtype 3 linewidth 5\n\
set style line 37 dashtype 4 linewidth 5\n\
set style line 38 dashtype 4 linewidth 5\n\
set style line 39 dashtype 4 linewidth 5\n\
set style line 40 dashtype 4 linewidth 5\n\
set style line 41 dashtype 4 linewidth 5\n\
set style line 42 dashtype 4 linewidth 5\n\
set style line 43 dashtype 4 linewidth 5\n\
set style line 44 dashtype 4 linewidth 5\n\
set style line 45 dashtype 4 linewidth 5\n\
set style line 46 dashtype 4 linewidth 5\n\
set style line 47 dashtype 4 linewidth 5\n\
set style line 48 dashtype 4 linewidth 5\n\
set style increment user"                                                                                                                                                                                                                                                                                                                                   );
  plot.GenerateOutput (file);
  file.close ();

	return 0;
}
