/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * Copyright (C) 2021 Christos Laskos
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "ftm-manager.h"
#include "ns3/core-module.h"
#include "ns3/wifi-mac-header.h"

#include <math.h>
#include <vector>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("FtmManager");

NS_OBJECT_ENSURE_REGISTERED (FtmManager);

TypeId
FtmManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FtmManager")
    .SetParent<Object> ()
    .SetGroupName ("Wifi")
    .AddConstructor<FtmManager>()
    ;
  return tid;
}

FtmManager::FtmManager ()
{
  received_packets = 0;
  awaiting_ack = false;
  sent_packets = 0;
  sending_ack = false;
}

FtmManager::FtmManager (Ptr<WifiPhy> phy, Ptr<Txop> txop, uint16_t channelWidth)
{
  received_packets = 0;
  awaiting_ack = false;
  sent_packets = 0;
  sending_ack = false;
  phy->TraceConnectWithoutContext("PhyFtmTxBegin", MakeCallback(&FtmManager::PhyFtmTxBegin, this));
  phy->TraceConnectWithoutContext("PhyFtmRxBegin", MakeCallback(&FtmManager::PhyFtmRxBegin, this));
  m_txop = txop;
  m_channelWidth = channelWidth;
  m_txPowerConst = 5.43*pow(10,-6);

  if (m_channelWidth == 20) {
    m_rssiThreshold = -70;
  } else if (m_channelWidth == 40) {
    m_rssiThreshold = -67;
  } else if (m_channelWidth == 80) {
    m_rssiThreshold = -64;
  } else {
    NS_FATAL_ERROR("Current FTM implementation not support for channel width " << m_channelWidth << " MHz!!");
  }

  m_preamble_detection_duration = phy->GetPreambleDetectionDuration();
}

FtmManager::~FtmManager ()
{
  TraceDisconnectWithoutContext("PhyFtmTxBegin", MakeCallback(&FtmManager::PhyFtmTxBegin, this));
  TraceDisconnectWithoutContext("PhyFtmRxBegin", MakeCallback(&FtmManager::PhyFtmRxBegin, this));
  sessions.clear();
  passiveSessions.clear();
  m_blocked_partners.clear();
  m_txop = 0;
}

void
FtmManager::PhyFtmTxBegin(Ptr<const Packet> packet)
{

  Time now = Simulator::Now();
  int64_t pico_sec = now.GetPicoSeconds();
  pico_sec &= 0x0000FFFFFFFFFFFF;
  sent_packets++;
  Ptr<Packet> copy = packet->Copy();
  WifiMacHeader hdr;
  copy->RemoveHeader(hdr);
  if(hdr.IsMgt() && hdr.IsAction()) {
      WifiActionHeader action_hdr;
      copy->RemoveHeader(action_hdr);
      if(action_hdr.GetCategory() == WifiActionHeader::PUBLIC_ACTION) {
          WifiActionHeader::ActionValue action = action_hdr.GetAction();
          if (action.publicAction == WifiActionHeader::FTM_RESPONSE)
            {
              Ptr<FtmSession> session = FindSession (hdr.GetAddr1());
              if (session != 0)
                {
                  FtmResponseHeader ftm_resp_hdr;
                  copy->RemoveHeader(ftm_resp_hdr);
                  // session->SetT1(ftm_resp_hdr.GetDialogToken(), pico_sec);

                  received_packets = 0;
                  awaiting_ack = true;

                  PacketInPieces pieces;
                  pieces.mac_hdr = hdr;
                  pieces.action_hdr = action_hdr;
                  pieces.ftm_res_hdr = ftm_resp_hdr;
                  m_current_tx_packet = pieces;
                }
            }
            broadcast_cnt += 1;
      }
  }
  else if(hdr.IsAck()) {
      if(sending_ack && sent_packets == 1) {
          if(m_ack_to == hdr.GetAddr1()) {
              sending_ack = false;
              Ptr<FtmSession> session = FindSession (m_current_rx_packet.mac_hdr.GetAddr2());
              if (session != 0)
                {
                  session->SetT3(m_current_rx_packet.ftm_res_hdr.GetDialogToken(), pico_sec);
                }
          }
      }
  }
}

void
FtmManager::PhyFtmRxBegin(Ptr<const Packet> packet, uint64_t tod)
{
  NS_LOG_FUNCTION (this);
  Time now = Simulator::Now();
  int64_t pico_sec = now.GetPicoSeconds();
  pico_sec &= 0x0000FFFFFFFFFFFF;
  Ptr<Packet> copy = packet->Copy();
  received_packets++;
  WifiMacHeader hdr;
  copy->RemoveHeader(hdr);
  if(hdr.GetAddr1() == m_mac_address || hdr.GetAddr1() == Mac48Address::GetBroadcast()){
      if(hdr.IsMgt() && hdr.IsAction()) {
          WifiActionHeader action_hdr;
          copy->RemoveHeader(action_hdr);
          if(action_hdr.GetCategory() == WifiActionHeader::PUBLIC_ACTION) {
              Mac48Address partner = hdr.GetAddr2();
              if(action_hdr.GetAction().publicAction == WifiActionHeader::FTM_RESPONSE) {
                  sending_ack = true;
                  sent_packets = 0;
                  m_ack_to = partner;

                  FtmResponseHeader ftm_res_hdr;
                  copy->RemoveHeader(ftm_res_hdr);

                  NS_LOG_DEBUG("Recieved TOD: " << tod);

                  Ptr<FtmSession> session = FindSession(partner);
                  if (session != 0 && ftm_res_hdr.GetDialogToken() != 0)
                    {
                      session->SetActiveTime(tod, pico_sec);
                      PacketInPieces pieces;
                      pieces.mac_hdr = hdr;
                      pieces.action_hdr = action_hdr;
                      pieces.ftm_res_hdr = ftm_res_hdr;
                      m_current_rx_packet = pieces;
                    }
              }
          }
      }
  }
}

void
FtmManager::SetMacAddress(Mac48Address addr)
{
  m_mac_address = addr;
}

Ptr<FtmSession>
FtmManager::CreateNewSession (Mac48Address partner, FtmSession::SessionType type, bool isPassive)
{
  if (isPassive) {
    if (FindPassiveSession(partner) == 0 && partner != m_mac_address)
    {
      Ptr<FtmSession> new_session = CreateObject<FtmSession> ();
      new_session->InitSession(partner, type, MakeCallback(&FtmManager::SendPacket, this));
      new_session->SetSessionOverCallbackManager(MakeCallback(&FtmManager::SessionOver, this));
      new_session->SetIsPassive();
      passiveSessions.insert({partner, new_session});
      return new_session;
    }
  } else { 
    if (FindSession(partner) == 0 && !CheckSessionBlocked (partner) && partner != m_mac_address)
    {
      Ptr<FtmSession> new_session = CreateObject<FtmSession> ();
      new_session->InitSession(partner, type, MakeCallback(&FtmManager::SendPacket, this));
      new_session->SetSessionOverCallbackManager(MakeCallback(&FtmManager::SessionOver, this));
      new_session->SetBlockSessionCallback(MakeCallback(&FtmManager::BlockSession, this));
      new_session->SetOverrideCallback(MakeCallback(&FtmManager::OverrideSession, this));
      new_session->SetPreambleDetectionDuration(m_preamble_detection_duration);
      sessions.insert({partner, new_session});
      return new_session;
    }
  }
  return 0;
}

void
FtmManager::SendPacket (Ptr<Packet> packet, WifiMacHeader hdr)
{
  hdr.SetType(WifiMacType::WIFI_MAC_MGT_ACTION);
  hdr.SetAddr2(m_mac_address);
  hdr.SetAddr3(m_mac_address);
  hdr.SetDsNotTo();
  hdr.SetDsNotFrom();
  //  hdr.SetNoRetry();

  uint8_t aifsn = 2 + (rand() % 4) + CalculateCurrentAifsn();

  m_txop->SetAifsn(aifsn);
  m_txop->Queue(packet, hdr);
}

uint8_t
FtmManager::CalculateCurrentAifsn (void)
{
  std::vector<int> receivedFtmCnts;

  std::map<Mac48Address, Ptr<FtmSession>>::iterator sessionIter;
  for (sessionIter=sessions.begin(); sessionIter != sessions.end(); sessionIter++) {
    if (sessionIter->first.IsBroadcast()) {
      continue;
    }
    receivedFtmCnts.push_back(sessionIter->second->GetFtmDialogs().size());
  }

  double R_ij = 0;
  int edgeCnt = 0;
  for (auto &cnt : receivedFtmCnts) {
    R_ij += log2(1+cnt);
    if (cnt != 0) {
      edgeCnt += 1;
    }
  }

  double dynamic_capture;
  if (broadcast_cnt == 0 && R_ij == 0) {
    dynamic_capture = 0;
  } else {
    dynamic_capture = (1+edgeCnt*log2(1+1/(1+broadcast_cnt)))/(broadcast_cnt+R_ij);
  }

  uint8_t aifsn = uint8_t(floor(log2(1+std::min(double(1024), 1024*dynamic_capture))));
  
  NS_LOG_DEBUG("Node addr : " << m_mac_address << ", received edges: " << 
               edgeCnt << ", R_ij: " << R_ij << ", broadcast_cnt: " << broadcast_cnt <<
               " acc_aifsn: " << int(aifsn) << ", time: " << Simulator::Now());



  return aifsn;
}

Ptr<FtmSession>
FtmManager::FindSession (Mac48Address addr)
{
  auto search = sessions.find(addr);
  if (search != sessions.end())
    {
      return search->second;
    }
  return 0;
}

double
FtmManager::FindDistance(Mac48Address addr)
{
  auto search = m_peerDistance.find(addr);
  if (search != m_peerDistance.end()) {
    return search->second;
  }

  NS_FATAL_ERROR("Cannot find distance by addr: ");
  return -1;

}

Ptr<FtmSession>
FtmManager::FindPassiveSession (Mac48Address addr)
{
  auto search = passiveSessions.find(addr);
  if (search != passiveSessions.end()) {
    return search->second;
  }

  return 0;
}

void
FtmManager::PassiveFTM (uint64_t timestamp)
{
  double distance = sqrt(m_txPowerConst/pow(10,(m_rssiThreshold-30)/10));
  std::map<Mac48Address, double>::iterator it;
  for (it = m_peerDistance.begin(); it != m_peerDistance.end(); it++) {
    if (it->second <= distance) {
      Ptr<FtmSession> session = FindPassiveSession(it->first);
      int64_t distanceTime = Seconds(it->second).GetPicoSeconds() * 2 / 299792458 + 2 * session->GetPreambleDetectionDuration();
      distanceTime &= 0x0000FFFFFFFFFFFF;
      session->SetPassiveTime(timestamp, timestamp + distanceTime);
    }
  }
}

void
FtmManager::SetPeerDistanceList(std::map<int, double> peerDistanceList, std::vector<Address> addrList)
{
  std::map<int, double>::iterator it;
  for (it = peerDistanceList.begin(); it != peerDistanceList.end(); it++) {
    m_peerDistance.insert({Mac48Address::ConvertFrom(addrList[it->first]), it->second});
  }
}

void
FtmManager::SessionOver (Mac48Address addr)
{
  sessions.erase (addr);
}

void
FtmManager::ReceivedFtmRequest (Mac48Address partner, FtmRequestHeader ftm_req)
{
  Ptr<FtmSession> session = FindSession (partner);
  if (session == 0)
    {
      session = CreateNewSession(partner, FtmSession::FTM_RESPONDER, false);
    }
  session->ProcessFtmRequest(ftm_req);
}

void
FtmManager::ReceivedFtmResponse (Mac48Address partner, FtmResponseHeader ftm_res)
{
  Ptr<FtmSession> session = FindSession (partner);
  if (session != 0)
    {
      session->ProcessFtmResponse(ftm_res);
    }
}


void
FtmManager::BlockSession (Mac48Address partner, Time duration)
{
  m_blocked_partners.push_back (partner);
  Simulator::Schedule(duration, &FtmManager::UnblockSession, this, partner);
}

void
FtmManager::UnblockSession (Mac48Address partner)
{
  m_blocked_partners.remove (partner);
}

bool
FtmManager::CheckSessionBlocked (Mac48Address partner)
{
  for (Mac48Address addr : m_blocked_partners)
    {
      if (addr == partner)
        {
          return true;
        }
    }
  return false;
}

void
FtmManager::OverrideSession (Mac48Address partner, FtmRequestHeader ftm_req)
{
  std::cout << "override" << std::endl;
  Ptr<FtmSession> session = CreateNewSession (partner, FtmSession::FTM_RESPONDER, false);
  session->ProcessFtmRequest (ftm_req);
}

}
