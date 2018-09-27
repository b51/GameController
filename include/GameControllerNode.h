/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GameControllerNode.h
*
*          Created On: Fri 05 Jan 2018 01:48:39 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef COMM_GAME_CONTROLLER_NODE_H_
#define COMM_GAME_CONTROLLER_NODE_H_

#include <stdio.h>
#include <iostream>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include <ros/ros.h>

#include <gflags/gflags.h>

#include "CommOptions.h"
#include "RoboCupGameControlData.h"
#include "UdpComm.h"

namespace IKid
{

class GameControllerNode
{
public:
  GameControllerNode(const CommOptions& options);
  ~GameControllerNode();
  void Init();
  void Run();

private:
  bool PacketReceive();
  bool PacketSend(const uint8_t& message);
  void UpdateShm();
  void HandleReceived();

  void ThreadReceive(int i);
  void ThreadSend(int i);
public:
  ::ros::NodeHandle node_handle_;
  ::ros::Publisher game_info_publisher_;

private:
  CommOptions options_;

  std::unique_ptr<UdpComm> udp_;
  in_addr game_ctrler_addr_;
  RoboCupGameControlData received_packet_;
};

} // namespace IKid

#endif
