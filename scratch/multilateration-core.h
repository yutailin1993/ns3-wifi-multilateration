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
#include <string>

using namespace ns3;

struct EnvConfig {
	std::size_t nAPs;
	std::size_t nSTAs;
	int mcs;
	int channelWidth;
};

struct UdpConfig {
	uint32_t payloadSize;
	std::string udpInterval;
};

typedef std::tuple<double, double, double> Position;
typedef std::vector<std::tuple<double, double, double>> PositionList;
typedef std::vector<Ptr<WifiNetDevice>> WifiNetDevicesList;
typedef std::vector<Address> AddressList;
typedef std::vector<Ptr<FtmSession>> SessionList;
typedef std::vector<std::tuple<size_t, size_t, double>> DistList;

/* variable definitation */

inline DistList ApStaDistList;
// std::string strChannelSettings = "{36, " + std::to_string(channelWidth) + ", BAND_5GHZ, 0}";

enum EModel {
	NO_ERROR,
	WIRED_ERROR,
	WIRELESS_ERROR
};

const PositionList APPositionCandidate = {
	{0, 0, 0},
	{20, 20, 0},
	{20, 0, 0},
	{0, 20, 0}
};

class WifiEnvironment
{
	public:
		WifiEnvironment(std::size_t in_nAPs, std::size_t in_nSTAs, int in_mcs, uint32_t in_payloadSize, double in_simulationTime, std::string in_udpInterval) {
			m_nSTAs = in_nSTAs;
			m_nAPs = in_nAPs;
			m_mcs = in_mcs;
			m_udpPort = 9;
			m_payloadSize = in_payloadSize;
			m_simulationTime = in_simulationTime;
			m_udpInterval = in_udpInterval;

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
		void SetupMobility();
		void SetupFTMEnv();
		void SetupApplication();

		WifiNetDevicesList GetWifiAPs();
		WifiNetDevicesList GetWifiSTAs();
		NodeContainer GetWifiNodes();
		AddressList GetRecvAddress();
		PositionList GetSTAPositions();

		ApplicationContainer GetServerApps();
		std::vector<ApplicationContainer> GetClientApps();

	private:
		int m_mcs;
		uint16_t m_udpPort;
		uint32_t m_payloadSize;
		double m_simulationTime;
		std::string m_udpInterval;

		PositionList m_apPositions;

		std::size_t m_nSTAs;
		std::size_t m_nAPs;
		bool m_enableRTSCTS;
		
		NodeContainer m_wifiNodes, m_wifiStaNodeGroups[3], m_wifiStaNodes, m_wifiApNodes;
		NetDeviceContainer m_devices, m_staDeviceGroups[3], m_staDevices, m_apDevices;
		
		YansWifiPhyHelper m_yansWifiPhy;
		YansWifiChannelHelper m_yansWifiChannel;
		WifiMacHelper m_wifiMac;
		WifiHelper m_wifi;

		MobilityHelper m_mobility;
		Ptr<ListPositionAllocator> m_apPosAlloc;
		Ptr<RandomDiscPositionAllocator> m_staPosAlloc;
		PositionList m_staPositions;

		InternetStackHelper m_stack;
		Ipv4InterfaceContainer m_staNodeGroupInterfaces[3], m_apNodeInterfaces;
		Ipv4AddressHelper m_ipv4Address;
		ApplicationContainer m_serverApp;
		std::vector<ApplicationContainer> m_clientApps;

		std::vector<Ptr<NetDevice>> m_APDevicesList, m_STADevicesList;
		WifiNetDevicesList m_wifiAPs, m_wifiSTAs;
		AddressList m_apAddrs, m_staAddrs;

		Ptr<NetDevice> GetDevice(bool in_getAPs, int in_deviceNo);
};

class Multilateration
{
	public:
		Multilateration(EModel e, int channelWidth) {
			m_errorModel = e;
			m_channelWidth = channelWidth;
			// m_ftmMap = LoadWirelessErrorMap();
		};

		virtual ~Multilateration();

		void SetFTMParams(int in_nBursts, double in_simulationTime);
		void ConstructAllSessions(EnvConfig in_envConf, WifiNetDevicesList in_APs, WifiNetDevicesList in_STAs, AddressList in_recvAddrs);

		SessionList GetAllSessions();

	private:
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
		Ptr<FtmSession> GenerateFTMSession(std::tuple<size_t, size_t> in_connection_Pair, Ptr<WifiNetDevice> in_STA, Address in_recvAddr);
};