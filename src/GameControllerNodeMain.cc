/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: GameControllerNodeMain.cc
*
*          Created On: Fri 05 Jan 2018 01:54:03 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "GameControllerNode.h"

using namespace ikid;
using namespace Comm;

DEFINE_string(configuration_directory, "",
                  "First directory in which configuration files are searched, "
                  "second is always the Cartographer installation to allow "
                  "including files from there.");
DEFINE_string(configuration_basename, "",
                  "Basename, i.e. not containing any directory prefix, of the "
                  "configuration file.");

/*****************************************
*
*  main function, all good things start
*
*****************************************/
int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  CommOptions comm_options =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  FLAGS_stderrthreshold = comm_options.log_level;

  ::ros::init(argc, argv, "GameControllerNode");
  ::ros::start();

  GameControllerNode game_ctrl_node(comm_options);
  game_ctrl_node.Init();
  game_ctrl_node.Run();

  ::ros::shutdown();

}
