/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: CommOptions.h
*
*          Created On: Sun 14 Jan 2018 08:36:28 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef CONFIG_COMM_OPTIONS_H_
#define CONFIG_COMM_OPTIONS_H_

#include <string>
#include <vector>
#include <tuple>
#include <glog/logging.h>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"

namespace IKid
{

struct CommOptions
{
  int log_level;
  int team_number;
  int player_number;
};

namespace carto = cartographer;

inline CommOptions CreateCommOptions(carto::common::LuaParameterDictionary* const
                                     lua_parameter_dictionary)
{
  CommOptions options;
  options.log_level = lua_parameter_dictionary->GetInt("log_level");

  options.team_number = lua_parameter_dictionary->GetInt("team_number");
  options.player_number = lua_parameter_dictionary->GetInt("player_number");

  return options;
}

inline CommOptions LoadOptions(const std::string& configuration_directory
                             , const std::string& configuration_basename)
{
  auto file_resolver = carto::common::make_unique<
      carto::common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateCommOptions(&lua_parameter_dictionary);
}

} // namespace IKid

#endif
