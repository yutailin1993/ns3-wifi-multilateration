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
#include "ns3/centralized-scheduler.h"

#include <vector>
#include <tuple>
#include <set>
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

inline DistList activeDistList;
inline DistList passiveDistList;
inline std::vector<int> DialogsCntList;
inline std::vector<std::tuple<size_t, size_t, std::list<int64_t>>> SessionRTTs;

enum EModel {
	NO_ERROR,
	WIRED_ERROR,
	WIRELESS_ERROR
};

enum DeviceType {
	AP,
	STA,
	ALL
};

const PositionList APPositionCandidate = {
	{0, 10, 0},
	{-10, 0, 0},
	{10, 0, 0},
	{0, -10, 0}
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
		void SetupDevicePhy(int64_t in_seed, std::string in_strChannelSettings);
		void SetRTSCTS(bool in_enableRTSCTS);
		void SetupMobility();
		void ConstructDeviceLists();
		void SetupApplication();
		void SetupCentralizedScheduler(double in_alpha,
																	 Time in_periodLength,
																	 TransmissionType in_transType,
																	 std::vector<std::vector<int>> in_independentSets);

		WifiNetDevicesList GetWifiAPs();
		WifiNetDevicesList GetWifiSTAs();
		NodeContainer GetWifiNodes();
		AddressList GetApAddress();
		AddressList GetStaAddress();
		PositionList GetStaPositions();
		PositionList GetApPositions();



		ApplicationContainer GetServerApps();
		std::vector<ApplicationContainer> GetClientApps();

	private:
		int m_mcs;
		uint16_t m_udpPort;
		uint32_t m_payloadSize;
		double m_simulationTime;
		std::string m_udpInterval;

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
		PositionList m_staPositions, m_apPositions;

		InternetStackHelper m_stack;
		Ipv4InterfaceContainer m_staNodeGroupInterfaces[3], m_apNodeInterfaces;
		Ipv4AddressHelper m_ipv4Address;
		ApplicationContainer m_serverApp;
		std::vector<ApplicationContainer> m_clientApps;

		std::vector<Ptr<NetDevice>> m_APDevicesList, m_STADevicesList, m_AllDevicesList;
		WifiNetDevicesList m_wifiAPs, m_wifiSTAs, m_wifiAll;
		AddressList m_apAddrs, m_staAddrs;
		Ptr<CentralizedScheduler> m_centralizedScheduler;

		Ptr<NetDevice> GetDevice(DeviceType in_deviceType, int in_deviceNo);
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

		void SetFTMParams(int in_nBursts, double in_simulationTime, int in_nSTAs, double in_alpha);
		void ConstructIfSessions(std::vector<int> in_anchorSTAs, WifiNetDevicesList in_APs, WifiNetDevicesList in_STAs, AddressList in_apAddrs);
		void ConstructPassiveSessions(EnvConfig in_envConf, WifiNetDevicesList in_STAs, AddressList in_staAddrs);
		void ConstructActiveSessions(EnvConfig in_envConf, WifiNetDevicesList in_STAs, AddressList in_staAddrs, std::set<std::vector<int>> in_activeLinks);
		void ConstructPeerDistance(EnvConfig in_envConf, WifiNetDevicesList in_STAs, PositionList in_posList, AddressList in_staAddrs);
		void EndAllSessions();

		SessionList GetAllSessions();

	private:
		int m_channelWidth;
		
		EModel m_errorModel;
		FtmParams m_ftmParams;
		Ptr<WiredFtmErrorModel> m_wiredErrorModel;
		SessionList m_sessionList;
		SessionList m_passiveSessionList;
		Ptr<WirelessFtmErrorModel::FtmMap> m_ftmMap;

		Ptr<WiredFtmErrorModel> GenerateWiredErrorModel();
		Ptr<WirelessFtmErrorModel> GenerateWirelessErrorModel(Ptr<WifiNetDevice> in_sta);
		WiredFtmErrorModel::ChannelBandwidth GetErrorModel();
		Ptr<WirelessFtmErrorModel::FtmMap> LoadWirelessErrorMap();
		Ptr<FtmSession> GenerateFTMSession(std::tuple<size_t, size_t> in_connection_Pair, Ptr<WifiNetDevice> in_STA, Address in_recvAddr);
		Ptr<FtmSession> GeneratePassiveFTMSession(std::tuple<size_t, size_t> in_connection_Pair, Ptr<WifiNetDevice> in_STA, Address in_recvAddr);
		std::map<int, double> ComputeDistance(size_t in_nodeIdx, int in_nSTAs, PositionList in_posLists);
		double DistanceCalculate(double x1, double y1, double x2, double y2);
};