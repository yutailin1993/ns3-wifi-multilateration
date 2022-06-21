#include <memory>

#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ssid.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/udp-client-server-helper.h"

using namespace ns3;

bool udp = true;
bool useRts = false;
bool useExtendedBlockAck = false;
bool enableUlOfdma = true;
bool enableBsrp = true;
double simulationTime = 5;
std::size_t maxNumStations = 20;

int contentionWindowList[3] = {15, 127, 1023};
int mcs = 5;
int channelWidth = 20;
int gi = 800;
uint32_t payloadSize = 1500;
int phyModel = 1;
int maxNRus = 4;
int ulPsduSize = 2000;
bool useCentral26toneRus = false;
bool forceDlOfdma = false;
uint16_t udpPort = 9;
std::string strChannelSettings = "{36, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

struct WifiPHYConfig
{
	int phyModel;
	int maxNRus;
	int ulPsduSize = 2000;
	bool forceDlOfdma;
	bool enableUlOfdma;
	bool enableTxopSharing = true;
	bool useCentral26toneRus = false;
	bool enableBsrp = false;
} WifiPHYConfig;
// std::string strScheduler = "Standard";

class WifiOfdmaSettings
{
public:
	WifiOfdmaSettings(std::size_t nStations, bool diffStaGroups, std::string dlAckSeqType) {
		m_nStations = nStations;
		m_clientApps = std::vector<ApplicationContainer>(nStations);
		m_diffStaGroups = diffStaGroups;
		m_ssid = Ssid("ns3-80211ax");
		m_dlAckSeqType = dlAckSeqType;
	};
	virtual ~WifiOfdmaSettings();

	void CreateNodes();
	void SetupDevicePhy(bool isIdealManager, struct WifiPHYConfig config, int64_t seed);
	void SetupMobility(double distance);
	void SetupMobility(double distanceList[]);
	void SetupApp();
	ApplicationContainer GetServerApp();
	std::vector<ApplicationContainer> GetClientApps();

private:
  std::size_t m_nStations;
	bool m_diffStaGroups;
	std::string m_dlAckSeqType;
	NodeContainer m_wifiStaNodes, m_wifiApNode;
	NodeContainer m_staNodeGroups[3];
	NetDeviceContainer m_apDevice, m_staDevices;
	WifiMacHelper m_mac;
	WifiHelper m_wifi;
	Ssid m_ssid;
	SpectrumWifiPhyHelper m_spectrumWifiPhy;
	// YansWifiPhyHelper m_yansWifiPhy;
	MobilityHelper m_mobility;
	InternetStackHelper m_stack;
	Ipv4AddressHelper m_address;
	Ipv4InterfaceContainer m_staNodeInterfaces, m_apNodeInterface;
	ApplicationContainer m_serverApp;
	std::vector<ApplicationContainer> m_clientApps;
	
};