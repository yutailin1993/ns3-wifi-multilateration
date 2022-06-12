#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/enum.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
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
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/gnuplot.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("he-wifi-network");

bool udp = true;
bool useRts = false;
bool useExtendedBlockAck = true;
bool enableUlOfdma = true;
bool enableBsrp = false;
double simulationTime = 5;
std::size_t maxNumStations = 10;

// Should be either NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR
std::string dlAckSeqTypeList[4] = {"NO-OFDMA", "ACK-SU-FORMAT", "MU-BAR", "AGGR-MU-BAR"};
int mcs = 7;
int channelWidth = 80;
double distance = 0;
int gi = 800;
uint32_t payloadSize = 1472;
int phyModel = 0;

void SetAckType(std::string dlAckSeqType)
{
	if (dlAckSeqType == "ACK-SU-FORMAT") {
    Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                        EnumValue (WifiAcknowledgment::DL_MU_BAR_BA_SEQUENCE));
  } else if (dlAckSeqType == "MU-BAR") {
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                          EnumValue (WifiAcknowledgment::DL_MU_TF_MU_BAR));
  } else if (dlAckSeqType == "AGGR-MU-BAR") {
      Config::SetDefault ("ns3::WifiDefaultAckManager::DlMuAckSequenceType",
                          EnumValue (WifiAcknowledgment::DL_MU_AGGREGATE_TF));
	} else if (dlAckSeqType != "NO-OFDMA") {
      NS_ABORT_MSG ("Invalid DL ack sequence type (must be NO-OFDMA, ACK-SU-FORMAT, MU-BAR or AGGR-MU-BAR)");
  }

	return;
}

double GetThroughput(ApplicationContainer serverApp)
{
	uint64_t rxBytes = 0;
	for (uint32_t i=0; i < serverApp.GetN(); i++) {
		rxBytes += payloadSize * DynamicCast<UdpServer> (serverApp.Get(i))->GetReceived();
	}

	double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbit/s

	return throughput;
}

double GetPacketLoss(ApplicationContainer serverApp, ApplicationContainer clientApps[])
{
	uint64_t rxPackets = 0, txPackets = 0;
	for (uint32_t i=0; i < serverApp.GetN(); i++) {
		rxPackets += DynamicCast<UdpServer> (serverApp.Get(i))->GetReceived();
		txPackets += DynamicCast<UdpClient> (clientApps[i].Get(0))->GetTotalTx() / payloadSize;
	}

	double packetLossRate = double (txPackets - rxPackets) / (double(txPackets)); // K Packets/s

	return packetLossRate;
}

void RunSimulation(std::string dlAckSeqType, Gnuplot &plot_throughput, Gnuplot &plot_packetLoss)
{
	Gnuplot2dDataset dataset_throughput = (dlAckSeqType);
	Gnuplot2dDataset dataset_packetLoss = (dlAckSeqType);

	std::cout << dlAckSeqType << std::endl;

	SetAckType(dlAckSeqType);
	if (dlAckSeqType != "NO-OFDMA") {
		phyModel = 1;
	}

	for (std::size_t nStations=1; nStations<=maxNumStations; nStations+=3) {
		NodeContainer wifiStaNodes;
		wifiStaNodes.Create(nStations);
		NodeContainer wifiApNode;
		wifiApNode.Create(1);

		NetDeviceContainer apDevice, staDevices;
		WifiMacHelper mac;
		WifiHelper wifi;
		std::string strChannelSettings = "{0, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

		wifi.SetStandard(WIFI_STANDARD_80211ax);

		std::ostringstream oss;
		oss << "HeMcs" << mcs;
		wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
																	"DataMode", StringValue(oss.str()),
																	"ControlMode", StringValue(oss.str()));

		Ssid ssid = Ssid("ns3-80211ax");

		if (phyModel == 1) {
			Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
			SpectrumWifiPhyHelper wifiPhy;
			// wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
			wifiPhy.SetChannel(spectrumChannel);


			mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
			wifiPhy.Set("Antennas", UintegerValue(1));
			wifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(1));
			wifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(1));
			wifiPhy.Set ("ChannelSettings", StringValue(strChannelSettings));

			staDevices = wifi.Install(wifiPhy, mac, wifiStaNodes);

			if (dlAckSeqType != "NO-OFDMA") {
				mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
																	"EnableUlOfdma", BooleanValue(enableUlOfdma));
																	// "EnableBsrp", BooleanValue(enableBsrp));
			}

			mac.SetType("ns3::ApWifiMac",
									"EnableBeaconJitter", BooleanValue(false),
									"Ssid", SsidValue(ssid));
			apDevice = wifi.Install(wifiPhy, mac, wifiApNode);
		} else {
			YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
			YansWifiPhyHelper wifiPhy;
			// wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
			wifiPhy.SetChannel(channel.Create());

			mac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid));
			wifiPhy.Set("Antennas", UintegerValue(1));
			wifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(1));
			wifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(1));
      wifiPhy.Set ("ChannelSettings", StringValue (strChannelSettings));
      staDevices = wifi.Install (wifiPhy, mac, wifiStaNodes);

      mac.SetType ("ns3::ApWifiMac",
                  "Ssid", SsidValue (ssid));
      apDevice = wifi.Install (wifiPhy, mac, wifiApNode);
		}

		// RngSeedManager::SetSeed(1);
		// RngSeedManager::SetRun(1);
		// int64_t streamNumber = 0;

		// streamNumber = wifi.AssignStreams(apDevice, streamNumber);
		// streamNumber = wifi.AssignStreams(staDevices, streamNumber);

		Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue(NanoSeconds(gi)));
		Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize", UintegerValue(useExtendedBlockAck ? 256 : 64));

		//Mobility
		MobilityHelper mobility;
		Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
		positionAlloc->Add (Vector (0.0, 0.0, 0.0));
		positionAlloc->Add (Vector (distance, 0.0, 0.0));
		mobility.SetPositionAllocator ((positionAlloc));

		mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

		mobility.Install(wifiApNode);
		mobility.Install(wifiStaNodes);

		InternetStackHelper stack;
		stack.Install(wifiApNode);
		stack.Install(wifiStaNodes);

		Ipv4AddressHelper address;
		address.SetBase("192.168.1.0", "255.255.255.0");
		Ipv4InterfaceContainer staNodeInterfaces;
		Ipv4InterfaceContainer apNodeInterface;

		staNodeInterfaces = address.Assign(staDevices);
		apNodeInterface = address.Assign(apDevice);

		ApplicationContainer serverApp;
		ApplicationContainer clientApps[nStations];

		uint16_t port = 9;
		UdpServerHelper server (port);
		serverApp = server.Install(wifiStaNodes);
		serverApp.Start (Seconds (0.0));
		serverApp.Stop (Seconds (simulationTime + 1));

		for (std::size_t i = 0; i < nStations; i++) {
			UdpClientHelper client (staNodeInterfaces.GetAddress (i), port);
			client.SetAttribute("MaxPackets", UintegerValue (4294967295u));
			client.SetAttribute("Interval", TimeValue (Time ("0.00001"))); //100,000 packets/s
			client.SetAttribute("PacketSize", UintegerValue(payloadSize));
			clientApps[i] = client.Install (wifiApNode.Get (0));
			clientApps[i].Start (Seconds (1.0));
			clientApps[i].Stop (Seconds (simulationTime + 1));
		}

		Simulator::Schedule (Seconds (0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

		Simulator::Stop(Seconds (simulationTime + 1));
		Simulator::Run ();

		double throughput = GetThroughput(serverApp);
		// double packetLoss = GetPacketLoss(serverApp, clientApps) / nStations;

		dataset_throughput.Add(nStations, throughput);
		// dataset_packetLoss.Add(nStations, packetLoss);

		std::cout << "Num of stations: " << nStations << ", total throughput: " << throughput << " Mbps" << std::endl;

		Simulator::Destroy(); 
	}

	plot_throughput.AddDataset(dataset_throughput);
	plot_packetLoss.AddDataset(dataset_packetLoss);

	return;
}

int main (int argc, char *argv[])
{
	std::ofstream file_throughput ("11ax-ofdma-Mcs7-throughput.plt");
	std::ofstream file_packetLoss ("11ax-ofdma-Mcs7-packetLoss.plt");

	Gnuplot plot_throughput = Gnuplot ("11ax-ofdma-Mcs7-throughput.eps");
	Gnuplot plot_packetLoss = Gnuplot ("11ax-ofdma-Mcs7-packetLoss.eps");

	if (useRts) {
		Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
	}

	for (std::string dlAckSeqType : dlAckSeqTypeList) {
		RunSimulation(dlAckSeqType, plot_throughput, plot_packetLoss);
	}
	
	plot_throughput.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  plot_throughput.SetLegend ("Stations numbers (#)", "Throughput (Mbit/s)");
  plot_throughput.SetExtra  ("set xrange [0:10]\n\
set yrange [0:800]\n\
set ytics 0,50,800\n\
set style line 1 dashtype 1 linewidth 5\n\
set style line 2 dashtype 1 linewidth 5\n\
set style line 3 dashtype 1 linewidth 5\n\
set style line 4 dashtype 1 linewidth 5\n\
set style increment user"                                                                                                                                                                                                                                                                                                                                   );
  plot_throughput.GenerateOutput (file_throughput);
  file_throughput.close ();

} 