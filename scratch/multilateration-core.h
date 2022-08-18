#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/application-container.h"
#include "ns3/wifi-net-device.h"
#include "ns3/ftm-header.h"
#include "ns3/ftm-session.h"
#include "ns3/mgt-headers.h"
#include "ns3/ftm-error-model.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"

#include <vector>
#include <tuple>

using namespace ns3;

struct EnvConfig {
	std::size_t nAPs;
	std::size_t nSTAs;
	int mcs;
	int channelWidth;
};

typedef std::tuple<double, double, double> Position;
typedef std::vector<std::tuple<double, double, double>> PositionList;
typedef std::vector<Ptr<WifiNetDevice>> WifiNetDevicesList;
typedef std::vector<Address> AddressList;
typedef std::vector<Ptr<FtmSession>> SessionList;

bool udp = true;
bool useRts = false;

int numSimulations = 20;
int channelWidth = 20;
int phyModel = 1;
uint16_t udpPort = 9;
double rss = -80;

std::string strChannelSettings = "{36, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

WifiStandard standard = WIFI_STANDARD_80211ax;

struct WifiPHYConfig
{
	int phyModel;
} WifiPHYConfig;

enum EModel {
	NO_ERROR,
	WIRED_ERROR,
	WIRELESS_ERROR
};

PositionList APPositionCandidate = {
	{0, 0, 0},
	{20, 20, 0},
	{20, 0, 0},
	{0, 20, 0}
};

class WifiEnvironment
{
	public:
		WifiEnvironment(std::size_t in_nAPs, std::size_t in_nSTAs, int in_mcs) {
			m_nSTAs = in_nSTAs;
			m_nAPs = in_nAPs;
			m_mcs = in_mcs;

			for (int i=0; i<in_nAPs; i++) {
				if (i > APPositionCandidate.size()) {
					NS_FATAL_ERROR ("Out of position candidates!");
				}

				m_apPositions.push_back(APPositionCandidate[i]);
			}
		};

		virtual ~WifiEnvironment();

		void CreateNodes();
		void SetupDevicePhy(int64_t in_seed);
		void SetRTSCTS(bool in_enableRTSCTS);
		void SetupMobility(Position in_staPosition);
		void SetupFTMEnv();

		WifiNetDevicesList GetWifiAPs();
		WifiNetDevicesList GetWifiSTAs();
		AddressList GetRecvAddress();

	private:
		int m_mcs;

		PositionList m_apPositions;

		std::size_t m_nSTAs;
		std::size_t m_nAPs;
		bool m_enableRTSCTS;
		
		NodeContainer m_wifiNodes;
		NetDeviceContainer m_devices;
		
		YansWifiPhyHelper m_yansWifiPhy;
		YansWifiChannelHelper m_yansWifiChannel;
		WifiMacHelper m_wifiMac;
		WifiHelper m_wifi;

		MobilityHelper m_mobility;
		Ptr<ListPositionAllocator> m_apPosAlloc;
		Ptr<ListPositionAllocator> m_staPosAlloc;

		std::vector<Ptr<NetDevice>> m_APDevices, m_STADevices;
		WifiNetDevicesList m_wifiAPs, m_wifiSTAs;
		AddressList m_recvAddrs;

		Ptr<NetDevice> GetDevice(bool in_getAPs, int in_deviceNo);
};

class Multilateration
{
	public:
		Multilateration(EModel e, int channelWidth) {
			m_errorModel = e;
			m_channelWidth = channelWidth;
			m_ftmMap = LoadWirelessErrorMap();
		};

		virtual ~Multilateration();

		void SetFTMParams(uint8_t in_nBursts);
		void ConstructAllSessions(EnvConfig in_envConf, WifiNetDevicesList in_APs, WifiNetDevicesList in_STAs, AddressList in_recvAddrs);

		SessionList GetAllSessions();

	private:
		uint8_t m_nBursts;
		int m_channelWidth;
		
		EModel m_errorModel;
		FtmParams m_ftmParams;
		Ptr<WiredFtmErrorModel> m_wiredErrorModel;
		SessionList m_sessionList;
		Ptr<WirelessFtmErrorModel::FtmMap> m_ftmMap;

		Ptr<WiredFtmErrorModel> GenerateWiredErrorModel();
		Ptr<WirelessFtmErrorModel> GenerateWirelessErrorModel(Ptr<WifiNetDevice> in_sta);
		WiredFtmErrorModel::ChannelBandwidth GetErrorModel();
		Ptr<WirelessFtmErrorModel::FtmMap> LoadWirelessErrorMap();
		Ptr<FtmSession> GenerateFTMSession(Ptr<WifiNetDevice> in_AP, Ptr<WifiNetDevice> in_STA, Address in_recvAddr);
};