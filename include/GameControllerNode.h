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

#include "GameController.h"

#include <stdio.h>
#include <iostream>
#include <chrono>

#include <ros/ros.h>

#include <gflags/gflags.h>

//#include "ikid_msgs/GameInfo.h"

namespace ikid
{

namespace Comm
{

class GameControllerNode
{
public:
  ::ros::NodeHandle node_handle_;
  ::ros::Publisher game_info_publisher_;

  CommOptions comm_options_;

  GameController *game_ctrl_;

public:
  GameControllerNode(const CommOptions& options);

  ~GameControllerNode();

  bool Init();

  void Run();
};

}

}

#endif
