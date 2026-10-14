/**
 * @file Settings.cpp
 * Implementation of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#include "Settings.h"
#include "Framework/LoggingTools.h"
#include "Platform/File.h"
#include "Streaming/InStreams.h"
#include "Streaming/Output.h"

#include "TextLogging/TextLogging.h"

DECL_TLOGGER(tlogger, "Settings", TextLogging::INFO);

Settings::Settings(const std::string& headName, const std::string& bodyName, float motionCycleTime, RobotType robotType) :
  headName(headName),
  bodyName(bodyName),
  motionCycleTime(motionCycleTime),
  robotType(robotType)
{
  InMapFile stream("settings.cfg");
  if(stream.exists())
    stream >> *this;
  else
    OUTPUT_ERROR("Could not load settings for robot \"" << headName.c_str() << "\" from settings.cfg");
  updateSearchPath();
}

Settings::Settings(const std::string& headName, const std::string& bodyName, float motionCycleTime, RobotType robotType,
                   int teamNumber, TeamColor fieldPlayerColor, TeamColor goalkeeperColor, int playerNumber,
                   const std::string& location, const std::string& scenario, unsigned char magicNumber) :
  headName(headName),
  bodyName(bodyName),
  motionCycleTime(motionCycleTime),
  robotType(robotType),
  teamNumber(teamNumber),
  fieldPlayerColor(fieldPlayerColor),
  goalkeeperColor(goalkeeperColor),
  playerNumber(playerNumber),
  location(location),
  scenario(scenario),
  magicNumber(magicNumber)
{
  updateSearchPath();
}

Settings::Settings(const std::string& logFileName, float motionCycleTime, RobotType robotType,
                   const std::string* location, const std::string* scenario) :
  Settings("Default", "Default", motionCycleTime, robotType)
{
  InBinaryFile stream(logFileName);
  if(stream.exists())
  {
    char magicByte;
    stream >> magicByte;
    if(magicByte == LoggingTools::logFileSettings)
    {
      LoggingTools::readSettings(stream, *this);
      goto settingsRead;
    }
  }
  LoggingTools::parseName(logFileName, &headName, &bodyName, &this->scenario, &this->location, nullptr, &playerNumber);
settingsRead:
  if(location)
    this->location = *location;
  if(scenario)
    this->scenario = *scenario;
  updateSearchPath();

  TLOGI(tlogger, "Settings from log file:");
  TLOGI(tlogger, "head={}, body={}, robotType={}", headName, bodyName, TypeRegistry::getEnumName(robotType));
  TLOGI(tlogger, "teamNumber={}, playerNumber={}, fieldPlayerColor={}, goalKeeperColor={}", teamNumber, playerNumber,
        TypeRegistry::getEnumName(fieldPlayerColor), TypeRegistry::getEnumName(goalkeeperColor));
  TLOGI(tlogger, "location={}, scenario={}", this->location, this->scenario);
  TLOGI(tlogger, "search path:");
  for (auto p : searchPath)
    TLOGI(tlogger, "    {}", p);
}

void Settings::updateSearchPath()
{
  searchPath.clear();
  searchPath.reserve(13);
  const std::string configDir = std::string(File::getBHDir()) + "/Config/";
  searchPath.push_back(configDir + "Robots/" + headName + "/Head/");
  searchPath.push_back(configDir + "Robots/" + bodyName + "/Body/");
  searchPath.push_back(configDir + "Robots/" + headName + "/" + bodyName + "/");
  std::string robotTypeName = TypeRegistry::getEnumName(robotType);
  robotTypeName[0] &= ~0x20;
  if(location != "Default")
  {
    if(robotType != nao)
      searchPath.push_back(configDir + "Locations/" + location + "/" + robotTypeName + "/");
    searchPath.push_back(configDir + "Locations/" + location + "/");
  }
  if(scenario != "Default")
  {
    if(robotType != nao)
      searchPath.push_back(configDir + "Scenarios/" + scenario + "/" + robotTypeName + "/");
    searchPath.push_back(configDir + "Scenarios/" + scenario + "/");
  }
  if(robotType != nao)
    searchPath.push_back(configDir + "Robots/Default/" + robotTypeName + "/");
  searchPath.push_back(configDir + "Robots/Default/");
  if(robotType != nao)
    searchPath.push_back(configDir + "Locations/Default/" + robotTypeName + "/");
  searchPath.push_back(configDir + "Locations/Default/");
  if(robotType != nao)
    searchPath.push_back(configDir + "Scenarios/Default/" + robotTypeName + "/");
  searchPath.push_back(configDir + "Scenarios/Default/");
}
