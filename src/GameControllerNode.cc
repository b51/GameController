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

using namespace ikid;
using namespace Comm;

GameControllerNode::GameControllerNode(const CommOptions& options):
  comm_options_(options)
{
  LOG(INFO) << "debug_mode       : " << comm_options_.debug_mode;
  LOG(INFO) << "team_number      : " << comm_options_.team_number;
  LOG(INFO) << "player_number    : " << comm_options_.player_number;
  //game_info_publisher_ = node_handle_.advertise<ikid_msgs::GameInfo>(comm_options_.game_info_topic, comm_options_.game_info_publisher_queue_size);

  game_ctrl_ = new GameController(comm_options_);
}

GameControllerNode::~GameControllerNode()
{
  if (game_ctrl_)
  {
    delete game_ctrl_;
    game_ctrl_ = nullptr;
  }
}

bool GameControllerNode::Init()
{
}

void GameControllerNode::Run()
{
  //ikid_msgs::GameCtrl info2pub;

  game_ctrl_->GameControllerPacketReceive();

  //info2pub.header.stamp = ::ros::Time::now();

  //game_info_publisher_.publish(info2pub);
}
