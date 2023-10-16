/**
 * @file Tools/Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#include "Settings.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Streams/InStreams.h"

#include "fmt/core.h"

Settings::Settings(const std::string& headName, const std::string& bodyName) :
  headName(headName),
  bodyName(bodyName)
{
  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");

#ifdef TARGET_ROBOT
  if(!overridePlayerNumber.empty())
  {
    for(const auto& mapping : overridePlayerNumber)
    {
      if(mapping.headName == headName)
      {
        playerNumber = mapping.playerNumber;
        return;
      }
    }
    FAIL(fmt::format("OverridePlayerNumber is not empty, but robot {} is not in there.", headName));
  }
#endif
}

Settings::Settings(const std::string &headName, const std::string &bodyName, int teamNumber, TeamColor fieldPlayerColor,
                   TeamColor goalkeeperColor, int playerNumber, const std::string &location,
                   const std::string &scenario, int teamPort, unsigned char magicNumber)
    : headName(headName), bodyName(bodyName), teamNumber(teamNumber), fieldPlayerColor(fieldPlayerColor),
      goalkeeperColor(goalkeeperColor), playerNumber(playerNumber), location(location), scenario(scenario),
      teamPort(teamPort), magicNumber(magicNumber)
{
}

Settings::Settings(const std::string &logFileName) : Settings("Default", "Default")
{
  LoggingTools::parseName(logFileName, nullptr, &headName, &bodyName, &scenario, &location, nullptr, &playerNumber);
}
