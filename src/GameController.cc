/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GameController.cc
*
*          Created On: Sun 14 Jan 2018 07:28:50 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "GameController.h"

using namespace ikid;
using namespace Comm;

GameController::GameController(const CommOptions& options)
  : options_(options)
{
  team_number_ = options.team_number;
  player_number_ = options.player_number_;

  Init();

  udp_ = new UdpComm();
  if(!udp_->setBlocking(false) ||
     !udp_->setBroadcast(true) ||
     !udp_->bind("0.0.0.0", GAMECONTROLLER_DATA_PORT) ||
     !udp_->setLoopback(false))
  {
    LOG(ERROR) << "Could not open UDP port";
    delete udp_;
    udp_ = nullptr;
  }

}

GameController::~GameController()
{
  if (udp_)
  {
    delete udp_;
    udp_ = nullptr;
  }
}

void GameController::Init()
{
  memset(&receive_packet_, 0, sizeof(receive_packet_));
  memset(&game_ctrler_addr_, 0, sizeof(game_ctrler_addr_));
}

bool GameController::PacketReceive()
{
  bool received = false;
  int size;
  RoboCupGameControlData buffer;
  struct sockaddr_in from;
  while(udp && (size = udp->read((char*) &buffer, sizeof(buffer), from)) > 0)
  {
    if (size == sizeof(buffer) &&
        !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
        buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
        teamNumber &&
        (buffer.teams[0].teamNumber == team_number_ ||
         buffer.teams[1].teamNumber == team_number_))
    {
      receive_packet_ = buffer;

      if (memcmp(&game_ctrler_addr_, &from.sin_addr, sizeof(in_addr)))
      {
        memcpy(&game_ctrler_addr_, &from.sin_addr, sizeof(in_addr));
        udp->setTarget(inet_ntoa(game_ctrler_addr_), GAMECONTROLLER_RETURN_PORT);
      }

      received = true;
    }
  }
  return received;
}

bool GameController::PacketSend(uint8_t message)
{
  RoboCupGameControlReturnData return_packet;
  return_packet.team = (uint8_t) team_number_;
  return_packet.player = (uint8_t) player_number_;
  return_packet.message = message;

  return !udp || udp->write((const char*) &return_packet, sizeof(return_packet));
}
