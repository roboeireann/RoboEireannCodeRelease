/**
 * @file Tools/Settings.h
 * Definition of a class that provides access to settings-specific configuration directories.
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Streams/Enum.h"
#include <string>

/**
 * @class Settings
 * The class provides access to settings-specific configuration directories.
 */
STREAMABLE(Settings,
{
public:
  /**
   * All allowed team colors. Their ordinals match the definitions of the
   * TEAM_* constants in the official header RoboCupGameControlData.h that
   * comes with the GameController.
   */
  ENUM(TeamColor,
  {,
    blue,
    red,
    yellow,
    black,
    white,
    green,
    orange,
    purple,
    brown,
    gray,
  });

  static_assert(TEAM_BLUE == blue && TEAM_RED == red && TEAM_YELLOW == yellow && TEAM_BLACK == black &&
                TEAM_WHITE == white && TEAM_GREEN == green && TEAM_ORANGE == orange &&
                TEAM_PURPLE == purple && TEAM_BROWN == brown && TEAM_GRAY == gray,
                "These macros and enums have to match!");

  STREAMABLE(HeadNameToPlayerNumber,
  {,
    (std::string) headName, /**< The name (head) of the robot. */
    (int) playerNumber, /**< The player number it should get. */
  });


  /**
   * Loads all settings except for the robot name from settings.cfg.
   * @param headName The name of the robot's head.
   * @param bodyName The name of the robot's body.
   */
  Settings(const std::string& headName, const std::string& bodyName);

  // TODO: will we allow goalkeeper number to be configured also???
  /**
   * Explicitly sets all settings.
   * @param headName The name of the robot's head.
   * @param bodyName The name of the robot's body.
   * @param teamNumber The team number.
   * @param fieldPlayerColor The team color.
   * @param goalkeeperColor The team color.
   * @param playerNumber The player number.
   * @param location The location.
   * @param scenario The scenario.
   * @param teamPort The team port.
   * @param magicNumber The magic number.
   */
  Settings(const std::string &headName, const std::string &bodyName, int teamNumber, TeamColor fieldPlayerColor,
           TeamColor goalkeeperColor, int playerNumber, const std::string &location, const std::string &scenario,
           int teamPort, unsigned char magicNumber);

  /**
   * Parses parts of the settings from a log file name and loads the rest from settings.cfg.
   * @param logFileName The name of the log file.
   */
  explicit Settings(const std::string& logFileName);

  /** return the default port number for team - BEWARE: needs change if SPL port allocation rules change */
  static int getDefaultPortForTeam(int teamNum) { return 10000 + teamNum; }

  /** return the port that has been configured, or the default port if not configured yet */
  int getPort() { return teamPort == -1 ? getDefaultPortForTeam(teamNumber) : teamPort; }


  static constexpr int lowestValidPlayerNumber = 1;  /**< No player can have a number smaller than this */
  // FIXME: hack to reduce the size of transmitted data since we never use numbers above 6
  static constexpr int highestValidPlayerNumber = 7; /* FIXME NUMROBOTS: MAX_NUM_PLAYERS; */ /**< No player can have a number greater than this */
  static constexpr int numPlayerNumbers = highestValidPlayerNumber - lowestValidPlayerNumber + 1;

  std::string headName; /**< The name of this robot's head. */
  std::string bodyName; /**< The name of this robot's body. */
  

  /* separate header from streamable fields, note comma after comment */,

  (int)(-1) teamNumber, /**< The number of our team in the game controller. Use theOwnTeamInfo.teamNumber instead. */
  (TeamColor)(numOfTeamColors) fieldPlayerColor, /**< The color of our team. Use theOwnTeamInfo value instead. */
  (TeamColor)(numOfTeamColors) goalkeeperColor, /**< The color of our goalie. Use theOwnTeamInfo value instead. */
  (int)(-1) playerNumber, /**< The number of the robot in the team. Use theRobotInfo.playerNumber instead. */
  (std::string)("Default") location, /**< The name of the location. */
  (std::string)("Default") scenario, /**< The name of the scenario. */
  (int)(-1) teamPort, /**< The UDP port our team uses for team communication. */
  (unsigned char)(0) magicNumber, /**< Magic Number for team communication. Assuring no foreign packets will be processed. */
  (std::vector<HeadNameToPlayerNumber>) overridePlayerNumber, /**< A mapping from head names to player numbers that overrides the playerNumber if not empty. */
});
