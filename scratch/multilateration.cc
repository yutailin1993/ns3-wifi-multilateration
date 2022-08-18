#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include "ns3/gnuplot.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/random-variable-stream.h"

#include "multilateration-core.h"

using namespace ns3;


Position staPos = {10, 7, 0};

void RunSession(Ptr<FtmSession> in_session)
{
	in_session->SessionBegin();
}

void RunSimulation(uint32_t in_seed, uint8_t in_nBursts, EModel in_e, EnvConfig in_envConf)
{
	RngSeedManager::SetSeed(in_seed);
	RngSeedManager::SetRun(in_seed);

	Config::SetDefault("ns3::WifiMac::FTM_Enabled", BooleanValue(true));

	WifiEnvironment wifiEnv = WifiEnvironment(in_envConf.nAPs, in_envConf.nSTAs, in_envConf.mcs);
	Multilateration positioning = Multilateration(in_e, in_envConf.channelWidth);


	wifiEnv.CreateNodes();
	wifiEnv.SetupDevicePhy(in_seed);
	wifiEnv.SetupMobility(staPos);
	wifiEnv.SetupFTMEnv();

	WifiNetDevicesList wifiAPlist = wifiEnv.GetWifiAPs();
	WifiNetDevicesList wifiSTAlist = wifiEnv.GetWifiSTAs();
	AddressList apAddrList = wifiEnv.GetRecvAddress();

	positioning.SetFTMParams(in_nBursts);
	positioning.ConstructAllSessions(in_envConf, wifiAPlist, wifiSTAlist, apAddrList);
	SessionList allSessions = positioning.GetAllSessions();
	int sessionsCount = allSessions.size();

	

	for (Ptr<FtmSession> session : allSessions) {
		Simulator::ScheduleNow(&RunSession, session);
	}
	
	Simulator::Stop(Seconds (10.0));
	Simulator::Run();
	Simulator::Destroy();
	

}

int main(int argc, char *argv[])
{
	Time::SetResolution(Time::PS);

	EnvConfig envConf = {3, 1, 4, 40};

	return 0;
}

