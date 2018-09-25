/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GameControllerNode.cc
*
*          Created On: Fri 05 Jan 2018 01:37:25 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "GameControllerNode.h"

namespace IKid
{

GameControllerNode::GameControllerNode(const CommOptions& options):
  options_(options)
{
  LOG(INFO) << "team_number      : " << options_.team_number;
  LOG(INFO) << "player_number    : " << options_.player_number;
}

GameControllerNode::~GameControllerNode()
{
  if (udp_)
    udp_.reset(nullptr);
}

void GameControllerNode::Init()
{
  memset(&receive_packet_, 0, sizeof(receive_packet_));
  memset(&game_ctrler_addr_, 0, sizeof(game_ctrler_addr_));

  udp_.reset(new UdpComm());

  if(!udp_->setBlocking(false) ||
     !udp_->setBroadcast(true) ||
     !udp_->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
     !udp_->setLoopback(false))
  {
    LOG(ERROR) << "Could not open UDP port";
    udp_.reset(nullptr);
  }
}

bool GameControllerNode::PacketReceive()
{
  bool received = false;
  int size;
  RoboCupGameControlData buffer;
  struct sockaddr_in from;
  while(udp_ && (size = udp_->read((char*) &buffer, sizeof(buffer), from)) > 0)
  {
    if (size == sizeof(buffer) &&
        !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
        buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
        options_.team_number &&
        (buffer.teams[0].teamNumber == options_.team_number ||
         buffer.teams[1].teamNumber == options_.team_number))
    {
      receive_packet_ = buffer;

      if (memcmp(&game_ctrler_addr_, &from.sin_addr, sizeof(in_addr)))
      {
        memcpy(&game_ctrler_addr_, &from.sin_addr, sizeof(in_addr));
        udp_->setTarget(inet_ntoa(game_ctrler_addr_), GAMECONTROLLER_RETURN_PORT);
      }

      received = true;
    }
  }
  if (received)
  {
    LOG(INFO) << receive_packet_.header;
    LOG(INFO) << receive_packet_.version;
    LOG(INFO) << receive_packet_.packetNumber;
    LOG(INFO) << receive_packet_.playersPerTeam;
    LOG(INFO) << receive_packet_.gameType;
    LOG(INFO) << receive_packet_.state;
    LOG(INFO) << receive_packet_.firstHalf;
    LOG(INFO) << receive_packet_.kickOffTeam;
    LOG(INFO) << receive_packet_.secondaryState;
    LOG(INFO) << receive_packet_.secondaryStateInfo;
    LOG(INFO) << receive_packet_.dropInTeam;
    LOG(INFO) << receive_packet_.secsRemaining;
    LOG(INFO) << receive_packet_.secondaryTime;
  }
  return received;
}

bool GameControllerNode::PacketSend(const uint8_t& message)
{
  RoboCupGameControlReturnData return_packet;
  return_packet.team = (uint8_t) options_.team_number;
  return_packet.player = (uint8_t) options_.player_number;
  return_packet.message = message;

  return !udp_ || udp_->write((const char*) &return_packet, sizeof(return_packet));
}

void GameControllerNode::UpdateShm()
{
}

void GameControllerNode::Run()
{
  while (node_handle_.ok())
  {
    PacketReceive();
    UpdateShm();
    usleep(10 * 1000);
  }
}

} // namespace IKid
