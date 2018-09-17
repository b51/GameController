/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GameController.h
*
*          Created On: Sun 14 Jan 2018 07:17:51 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef COMM_GAME_CONTROLLER_H_
#define COMM_GAME_CONTROLLER_H_

#include <glog/logging.h>

#include "Config/CommOptions.h"

#include "RoboCupGameControlData.h"
#include "UdpComm.h"

namespace ikid
{

namespace Comm
{

class GameController
{
public:
  UdpComm *udp_;
  in_addr game_ctrler_addr_;

  RoboCupGameControlData receive_packet_;

  int team_number_;
  int player_number_;

public:
  GameController(const CommOptions& options);
  ~GameController();

  void Init();

  int PacketParse();
  bool PacketReceive();
  bool PacketSend(uint8_t message);

};

}

}

#endif
