#include <string>

#include "ns3/pointer.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/boolean.h"

#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/qos-txop.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/ap-wifi-mac.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/rng-seed-manager.h"


#include "ns3/config.h"
#include "ns3/enum.h"
#include "ns3/wifi-acknowledgment.h"
#include "ns3/wifi-net-device.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/gnuplot.h"

#include "wifi-11ax-ofdma.h"

using namespace ns3;

const std::string dlAckSeqTypeList[4] = {"ACK-SU-FORMAT", "MU-BAR", "AGGR-MU-BAR", "NO-OFDMA"};
double distance = 0;
double distanceSet[3] = {0, 50, 100};
std::string expName = "11ax-ofdma-ACK-nSTA-Jitter-Ideal";
std::string udpInterval = "0.00012";
bool enableJitter = true;
bool useIdeal = false;
bool diffGroup = false;

WifiOfdmaSettings::~WifiOfdmaSettings()
{
	m_clientApps.clear();
}

void
WifiOfdmaSettings::CreateNodes()
{
	m_wifiApNode.Create(1);
	if (!m_diffStaGroups) {
		m_wifiStaNodes.Create(m_nStations);
	} else {
		for (auto &staNodes : m_staNodeGroups) {
			staNodes.Create(m_nStations / 3);
			m_wifiStaNodes.Add(staNodes);
		}
	}
	
	return;
}

void
WifiOfdmaSettings::SetupDevicePhy(bool isIdealManager, struct WifiPHYConfig config, int64_t seed)
{
	m_wifi.SetStandard(WIFI_STANDARD_80211ax);
	std::ostringstream oss;
	oss<< "HeMcs" << mcs;

	if (isIdealManager) {
		m_wifi.SetRemoteStationManager("ns3::IdealWifiManager");
	} else {
		m_wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
																	 "DataMode", StringValue(oss.str()),
																	 "ControlMode", StringValue("OfdmRate24Mbps"));
	}

	Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();
	Ptr<FriisPropagationLossModel> lossModel = CreateObject<FriisPropagationLossModel>();
	spectrumChannel->AddPropagationLossModel(lossModel);
	Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel>();
	spectrumChannel->SetPropagationDelayModel (delayModel);
	
	m_spectrumWifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
	// m_spectrumWifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
	m_spectrumWifiPhy.SetChannel(spectrumChannel);
	m_spectrumWifiPhy.Set("TxPowerStart", DoubleValue(17.0));
	m_spectrumWifiPhy.Set("TxPowerEnd", DoubleValue(17.0));
	m_spectrumWifiPhy.Set("Antennas", UintegerValue(1));
	m_spectrumWifiPhy.Set("MaxSupportedTxSpatialStreams", UintegerValue(1));
	m_spectrumWifiPhy.Set("MaxSupportedRxSpatialStreams", UintegerValue(1));
	m_spectrumWifiPhy.Set ("ChannelSettings", StringValue(strChannelSettings));
	m_spectrumWifiPhy.Set ("TxGain", DoubleValue (0.0));
 	m_spectrumWifiPhy.Set ("RxGain", DoubleValue (0.0));
 	m_spectrumWifiPhy.Set ("RxSensitivity", DoubleValue (-91.0));

	m_mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(m_ssid));
	m_staDevices = m_wifi.Install(m_spectrumWifiPhy, m_mac, m_wifiStaNodes);
	
	if (m_dlAckSeqType != "NO-OFDMA") {
		m_mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler",
															"NStations", UintegerValue(config.maxNRus),
															"ForceDlOfdma", BooleanValue(config.forceDlOfdma),
															"EnableUlOfdma", BooleanValue(config.enableUlOfdma),
															"EnableTxopSharing", BooleanValue(config.enableTxopSharing),
															"UlPsduSize", UintegerValue(config.ulPsduSize),
															"UseCentral26TonesRus", BooleanValue(config.useCentral26toneRus),
															"EnableBsrp", BooleanValue(config.enableBsrp)); // "SchedulerLogic", StringValue(strScheduler));
	}

	m_mac.SetType("ns3::ApWifiMac",
								"EnableBeaconJitter", BooleanValue(enableJitter),
								"Ssid", SsidValue(m_ssid));
	m_apDevice = m_wifi.Install(m_spectrumWifiPhy, m_mac, m_wifiApNode);

	uint64_t streamNumber = 1;
	streamNumber += m_wifi.AssignStreams(m_apDevice, seed);
	streamNumber += m_wifi.AssignStreams(m_staDevices, streamNumber);
	
	Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (m_apDevice.Get(0));
	dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(0));
	dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(10060));
	dev->GetMac()->SetAttribute("BeaconInterval", TimeValue(NanoSeconds(102400000 * 20)));

	PointerValue ptr;

	dev->GetMac()->GetAttribute("BE_Txop", ptr);
	ptr.Get<QosTxop> ()->SetTxopLimit(MicroSeconds(4800));
 	ptr.Get<QosTxop> ()->GetWifiMacQueue ()->SetMaxDelay (MilliSeconds (20000));
	ptr.Get<QosTxop> ()->SetMinCw (15);
 	// ptr.Get<QosTxop> ()->GetBaManager ()->GetRetransmitQueue ()->SetMaxDelay (MilliSeconds (20000));

	for (std::size_t i=0; i<m_nStations; i++) {
		dev = DynamicCast<WifiNetDevice> (m_staDevices.Get(i));
		dev->GetMac()->SetAttribute("BE_MaxAmsduSize", UintegerValue(0));
		dev->GetMac()->SetAttribute("BE_MaxAmpduSize", UintegerValue(10060));

		dev->GetMac()->GetAttribute("BE_Txop", ptr);
		ptr.Get<QosTxop> ()->SetTxopLimit(MicroSeconds(4800));
 		ptr.Get<QosTxop> ()->GetWifiMacQueue ()->SetMaxDelay (MilliSeconds (20000));
		ptr.Get<QosTxop> ()->SetMinCw (15);
 		// ptr.Get<QosTxop> ()->GetBaManager ()->GetRetransmitQueue ()->SetMaxDelay (MilliSeconds (20000));

	}
}

void
WifiOfdmaSettings::SetupMobility(double distance)
{
	Vector apPos (0.0, 0.0, 0.0);
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add (apPos);
	m_mobility.SetPositionAllocator(positionAlloc);
	m_mobility.Install(m_wifiApNode);

	Ptr<UniformDiscPositionAllocator> staPositionAlloc =
		CreateObjectWithAttributes<UniformDiscPositionAllocator> ("rho", DoubleValue(distance),
																															"X", DoubleValue(apPos.x),
																															"Y", DoubleValue(apPos.y));
	m_mobility.SetPositionAllocator (staPositionAlloc);
	m_mobility.Install (m_wifiStaNodes);
}

void
WifiOfdmaSettings::SetupMobility(double distanceList[])
{
	Vector apPos (0.0, 0.0, 0.0);
	Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
	positionAlloc->Add (apPos);
	m_mobility.SetPositionAllocator(positionAlloc);
	m_mobility.Install(m_wifiApNode);

	for (int i = 0; i<3; i++) {
		Ptr<UniformDiscPositionAllocator> staPositionAlloc =
			CreateObjectWithAttributes<UniformDiscPositionAllocator> ("rho", DoubleValue(distanceList[i]),
																																"X", DoubleValue(apPos.x),
																																"Y", DoubleValue(apPos.y));
		m_mobility.SetPositionAllocator (staPositionAlloc);
		m_mobility.Install(m_staNodeGroups[i]);
	}
}

void
WifiOfdmaSettings::SetupApp()
{
	m_stack.Install(m_wifiApNode);
	m_stack.Install(m_wifiStaNodes);

	m_address.SetBase("192.168.1.0", "255.255.255.0");

	m_staNodeInterfaces = m_address.Assign(m_staDevices);
	m_apNodeInterface = m_address.Assign(m_apDevice);

	uint16_t port = 9;
	UdpServerHelper server(port);
	m_serverApp = server.Install(m_wifiStaNodes);
	m_serverApp.Start (Seconds (0.0));
	m_serverApp.Stop (Seconds (simulationTime + 1));

	for (std::size_t i=0; i<m_nStations; i++) {
		UdpClientHelper client (m_staNodeInterfaces.GetAddress(i), port);
		client.SetAttribute("MaxPackets", UintegerValue (4294967295u));
		client.SetAttribute("Interval", TimeValue (Time (udpInterval))); //1,000,000 packets/s
		client.SetAttribute("PacketSize", UintegerValue(payloadSize));

		m_clientApps[i] = client.Install(m_wifiApNode.Get(0));
		m_clientApps[i].Start(Seconds (1.0));
		m_clientApps[i].Stop(Seconds (simulationTime + 1));
	}
}

ApplicationContainer WifiOfdmaSettings::GetServerApp()
{
	return m_serverApp;
}

std::vector<ApplicationContainer> WifiOfdmaSettings::GetClientApps()
{
	return m_clientApps;
}


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

void PrintStaNodesPos(NodeContainer wifiStaNodes)
{
	for (std::size_t i=0; i != wifiStaNodes.GetN(); i++) {
		Ptr<MobilityModel> mob = wifiStaNodes.Get(i)->GetObject<MobilityModel>();
		std::cout << "Sta " << i << ": " << "(" << mob->GetPosition().x << ", " \
							<< mob->GetPosition().y << ", " << mob->GetPosition().z << ")" << std::endl;
	}
}

double GetThroughput(ApplicationContainer serverApp)
{
	uint64_t rxBytes = 0;
	for (uint32_t i=0; i < serverApp.GetN(); i++) {
		rxBytes += payloadSize * DynamicCast<UdpServer> (serverApp.Get(i))->GetReceived();
	}

	double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); //Mbits/s

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

double GetStaGroupThroughput(ApplicationContainer serverApp, int groupID, int nStations)
{
	uint64_t rxBytes = 0;
	uint32_t groupSize = nStations / 3;
	for (uint32_t i=groupID*groupSize; i < (groupID + 1)*groupSize; i++) {
		rxBytes += payloadSize * DynamicCast<UdpServer> (serverApp.Get(i))->GetReceived();
	}

	double throughput = (rxBytes * 8) / (simulationTime * 1000000.0); // Mbits/s

	return throughput;
}

void RunSimulation(std::string dlAckSeqType, Gnuplot &plot_throughput)
{
	Gnuplot2dDataset dataset_throughput = (dlAckSeqType);
	Gnuplot2dDataset dataset_packetLoss = (dlAckSeqType);

	std::cout << dlAckSeqType << std::endl;

	SetAckType(dlAckSeqType);

	for (std::size_t nStations=1; nStations<=maxNumStations; nStations+=4) {
		struct WifiPHYConfig conf;
		if (dlAckSeqType != "NO-OFDMA") {
			conf.phyModel = 1;
			conf.maxNRus = nStations;
			conf.forceDlOfdma = true;
			conf.enableUlOfdma = true;
		} else {
			conf.phyModel = 1;
			conf.maxNRus = 1;
			conf.forceDlOfdma = false;
			conf.enableUlOfdma = false;
		}

		double all_throughput = 0.0;

		for (uint64_t seed=1; seed <= 10; seed++) {
			WifiOfdmaSettings ofdmaTest = WifiOfdmaSettings(nStations, false, dlAckSeqType);
			ofdmaTest.CreateNodes();
			ofdmaTest.SetupDevicePhy(useIdeal, conf, seed);
			ofdmaTest.SetupMobility(distance);
			// ofdmaTest.SetupMobility(distanceSet);
			ofdmaTest.SetupApp();

			Simulator::Schedule (Seconds (0), &Ipv4GlobalRoutingHelper::PopulateRoutingTables);

			Simulator::Stop(Seconds (simulationTime + 1));
			Simulator::Run ();

			ApplicationContainer serverApp = ofdmaTest.GetServerApp();

			all_throughput += GetThroughput(serverApp);
			// double packetLoss = GetPacketLoss(serverApp, clientApps) / nStations;

			// for (int i=0; i<3; i++) {
			// 	double throughput = GetStaGroupThroughput(serverApp, i, nStations);
			// 	dataset_throughput.Add(distanceSet[i], throughput);
			// 	std::cout << "Group dist: " << distanceSet[i] << ", group avg throughput: " << throughput << " Mbps" << std::endl;
			// }

			Simulator::Destroy();
		}
		all_throughput /= 10;
		dataset_throughput.Add(nStations, all_throughput);
		// dataset_packetLoss.Add(nStations, packetLoss);
		std::cout << "Num of stations: " << nStations << ", total throughput: " << all_throughput << " Mbps" << std::endl;
	}

	plot_throughput.AddDataset(dataset_throughput);

	return;
}

int main (int argc, char *argv[])
{
	std::ofstream file_throughput (expName + ".plt");
	// std::ofstream file_packetLoss ("11ax-ofdma-ACK-nSTA-delay.plt");

	Gnuplot plot_throughput = Gnuplot (expName + ".eps");
	// Gnuplot plot_packetLoss = Gnuplot ("11ax-ofdma-ACK-nSTA-delay.eps");

	if (useRts) {
		Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("0"));
	}

	for (std::string dlAckSeqType : dlAckSeqTypeList) {
		RunSimulation(dlAckSeqType, plot_throughput);
	}
	
	plot_throughput.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  plot_throughput.SetLegend ("Stations numbers (#)", "Throughput (Mbit/s)");
  plot_throughput.SetExtra  ("set xrange [0:20]\n\
set yrange [0:60]\n\
set ytics 10,5,60\n\
set xtics 1,4,20\n\
set style line 1 dashtype 1 linewidth 5 pt 1 ps 1\n\
set style line 2 dashtype 2 linewidth 5 pt 2 ps 1\n\
set style line 3 dashtype 3 linewidth 5 pt 3 ps 1\n\
set style line 4 dashtype 4 linewidth 5 pt 4 ps 1\n\
set style increment user\n\
plot \"-\"  title \"NO-OFDMA\" with linespoints linestyle 1, \"-\"  title \"ACK-SU-FORMAT\" with linespoints linestyle 2, \"-\"  title \"MU-BAR\" with linespoints linestyle 3, \"-\"  title \"AGGR-MU-BAR\" with linespoints linestyle 4"                                                                                                                                                                                                                                                                                                                            );
  plot_throughput.GenerateOutput (file_throughput);
  file_throughput.close ();

} 