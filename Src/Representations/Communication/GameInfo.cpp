/**
 * @file GameInfo.cpp
 * 
 * The file implements a struct that encapsulates the structure RoboCupGameControlData
 * defined in the file RoboCupGameControlData.h that is provided with the GameController.
 * 
 * @author Thomas Röfer
 */

#include "GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Debugging/ColorRGBA.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Global.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"
#include <algorithm>


///////////////////////////////////////////////////////////////////////////////
// nested RobotInfo
///////////////////////////////////////////////////////////////////////////////

void ReceivedGameControlData::RobotInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(RobotInfo);
  // Note: must match the fields in RoboCup::RobotInfo
  REG(penalty);
  REG(secsTillUnpenalised);
}

/**
 * Write a robot info to a stream.
 */
Out& operator<<(Out& stream, const ReceivedGameControlData::RobotInfo& robotInfo)
{
  STREAM_EXT(stream, robotInfo.penalty);
  STREAM_EXT(stream, robotInfo.secsTillUnpenalised);
  return stream;
}

/**
 * Read a robot info from a stream.
 */
In& operator>>(In& stream, ReceivedGameControlData::RobotInfo& robotInfo)
{
  STREAM_EXT(stream, robotInfo.penalty);
  STREAM_EXT(stream, robotInfo.secsTillUnpenalised);
  return stream;
}

///////////////////////////////////////////////////////////////////////////////
// nested TeamInfo
///////////////////////////////////////////////////////////////////////////////

int ReceivedGameControlData::TeamInfo::getSubstitutedPlayerNumber(int number) const
{
  if(number < 6)
    return number;

  for(unsigned int i = 0; i < 5; i++)
    if(players[i].penalty == PENALTY_SUBSTITUTE)
      return i + 1;

  return number;
}


void ReceivedGameControlData::TeamInfo::reg()
{
  PUBLISH(reg);
  REG_CLASS(TeamInfo);

  // Note: must match the fields in RoboCup::TeamInfo

  REG(teamNumber);
  REG(fieldPlayerColor);
  REG(goalkeeperColor);
  REG(score);
  REG(penaltyShot);
  REG(singleShots);
  REG(messageBudget);
  REG(RobotInfo(&)[MAX_NUM_PLAYERS], players);
}

/**
 * Write a team info to a stream.
 */
Out& operator<<(Out& stream, const ReceivedGameControlData::TeamInfo& teamInfo)
{
  const ReceivedGameControlData::RobotInfo(&players)[MAX_NUM_PLAYERS] =
      reinterpret_cast<const ReceivedGameControlData::RobotInfo(&)[MAX_NUM_PLAYERS]>(teamInfo.players);
  STREAM_EXT(stream, teamInfo.teamNumber);
  STREAM_EXT(stream, teamInfo.fieldPlayerColor);
  STREAM_EXT(stream, teamInfo.goalkeeperColor);
  STREAM_EXT(stream, teamInfo.score);
  STREAM_EXT(stream, teamInfo.penaltyShot);
  STREAM_EXT(stream, teamInfo.singleShots);
  STREAM_EXT(stream, teamInfo.messageBudget);
  STREAM_EXT(stream, players);
  return stream;
}

/**
 * Read a team info from a stream.
 */
In& operator>>(In& stream, ReceivedGameControlData::TeamInfo& teamInfo)
{
  ReceivedGameControlData::RobotInfo(&players)[MAX_NUM_PLAYERS] =
      reinterpret_cast<ReceivedGameControlData::RobotInfo(&)[MAX_NUM_PLAYERS]>(teamInfo.players);
  STREAM_EXT(stream, teamInfo.teamNumber);
  STREAM_EXT(stream, teamInfo.fieldPlayerColor);
  STREAM_EXT(stream, teamInfo.goalkeeperColor);
  STREAM_EXT(stream, teamInfo.score);
  STREAM_EXT(stream, teamInfo.penaltyShot);
  STREAM_EXT(stream, teamInfo.singleShots);
  STREAM_EXT(stream, teamInfo.messageBudget);
  STREAM_EXT(stream, players);
  return stream;
}


///////////////////////////////////////////////////////////////////////////////
// ReceivedGameControlData itself
///////////////////////////////////////////////////////////////////////////////

ReceivedGameControlData::ReceivedGameControlData()
{
  memset(static_cast<RoboCup::RoboCupGameControlData*>(this), 0, sizeof(RoboCup::RoboCupGameControlData));
}

void ReceivedGameControlData::reg()
{
  PUBLISH(reg);
  REG_CLASS(ReceivedGameControlData);
  // must match the fields and order in RoboCup::RoboCupGameControlData with
  // our additions at the end
  REG(packetNumber);
  REG(playersPerTeam);
  REG(competitionPhase);
  REG(competitionType);
  REG(gamePhase);
  REG(state);
  REG(setPlay);
  REG(firstHalf);
  REG(kickingTeam);
  REG(secsRemaining);
  REG(secondaryTime);
  REG(TeamInfo(&)[2], teams);

  REG(timeLastPacketReceived);
}



void ReceivedGameControlData::read(In& stream)
{
  TeamInfo(&teams)[2] = reinterpret_cast<TeamInfo(&)[2]>(this->teams);

  STREAM(packetNumber);
  STREAM(playersPerTeam);
  STREAM(competitionPhase); // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType);  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_GENERAL_PENALTY_KICK)
  STREAM(gamePhase); // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickingTeam); // team number
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime);
  STREAM(teams);

  STREAM(timeLastPacketReceived); // used to decide whether a gameController is running
}

void ReceivedGameControlData::write(Out& stream) const
{
  const TeamInfo(&teams)[2] = reinterpret_cast<const TeamInfo(&)[2]>(this->teams);

  STREAM(packetNumber);
  STREAM(playersPerTeam);
  STREAM(competitionPhase); // phase of the competition (COMPETITION_PHASE_ROUNDROBIN, COMPETITION_PHASE_PLAYOFF)
  STREAM(competitionType);  // type of the competition (COMPETITION_TYPE_NORMAL, COMPETITION_TYPE_GENERAL_PENALTY_KICK)
  STREAM(gamePhase); // phase of the game (GAME_PHASE_NORMAL, GAME_PHASE_PENALTYSHOOT, etc)
  STREAM(state); // STATE_READY, STATE_PLAYING, ...
  STREAM(setPlay); // active set play (SET_PLAY_NONE, SET_PLAY_GOAL_KICK, etc)
  STREAM(firstHalf); // 1 = game in first half, 0 otherwise
  STREAM(kickingTeam); // team number
  STREAM(secsRemaining); // estimate of number of seconds remaining in the half.
  STREAM(secondaryTime);
  STREAM(teams);

  STREAM(timeLastPacketReceived); // used to decide whether a gameController is running
}


std::string ReceivedGameControlData::getStateString() const
{
  switch(state)
  {
    case STATE_INITIAL:
      return "Initial";
    case STATE_STANDBY:
      return "Standby";
    case STATE_READY:
      if(setPlay == SET_PLAY_PENALTY_KICK)
        return "Ready (Penalty Kick)";
      else
        return "Ready";
    case STATE_SET:
      if(setPlay == SET_PLAY_PENALTY_KICK)
        return "Set (Penalty Kick)";
      else
        return "Set";
    case STATE_PLAYING:
      switch(setPlay)
      {
        case SET_PLAY_NONE:
          return "Playing";
        case SET_PLAY_GOAL_KICK:
          return "Goal Kick";
        case SET_PLAY_PUSHING_FREE_KICK:
          return "Pushing Free Kick";
        case SET_PLAY_CORNER_KICK:
          return "Corner Kick";
        case SET_PLAY_KICK_IN:
          return "Kick In";
        case SET_PLAY_PENALTY_KICK:
          return "Penalty Kick";
        default:
          return "Unknown";
      }
    case STATE_FINISHED:
      return "Finished";
    default:
      return "Unknown";
  }
}


std::string ReceivedGameControlData::getPenaltyString() const
{
  switch (playerInfo().penalty)
  {
    case PENALTY_SPL_ILLEGAL_BALL_CONTACT: return "Illegal Ball Contact";
    case PENALTY_SPL_PLAYER_PUSHING: return "Player Pushing";
    case PENALTY_SPL_ILLEGAL_MOTION_IN_SET: return "Illegal Motion in Set";
    case PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY: return "Illegal Motion in Standby";
    case PENALTY_SPL_INACTIVE_PLAYER: return "Inactive Player";
    case PENALTY_SPL_ILLEGAL_POSITION: return "Illegal Position";
    case PENALTY_SPL_LEAVING_THE_FIELD: return "Leaving the Field";
    case PENALTY_SPL_REQUEST_FOR_PICKUP: return "Request for Pickup";
    case PENALTY_SPL_LOCAL_GAME_STUCK: return "Local Game Stuck";
    case PENALTY_SPL_ILLEGAL_POSITION_IN_SET: return "Illegal Position in Set";
    case PENALTY_SUBSTITUTE: return "Substitute";
    case PENALTY_MANUAL: return "Manual";
    default: return "None";
  }
}

std::string ReceivedGameControlData::getPlayerNumberAndColorString() const
{
  // Note we believe our settings in preference to the GC info for colour
  return fmt::format("{} {}",
                     TypeRegistry::getEnumName(isGoalkeeper() ? Global::getSettings().goalkeeperColor
                                                              : Global::getSettings().fieldPlayerColor),
                     toPlayerNumber(playerIndex()));
}




static void drawChar(int character, const Vector3f& pos, float size, const ColorRGBA& color)
{
  static const Vector3f points[8] =
  {
    Vector3f(1, 0, 1), //  1
    Vector3f(1, 0, 0), // 0 2
    Vector3f(0, 0, 0), //  6
    Vector3f(0, 0, 1), // 5 3
    Vector3f(0, 0, 2), //  4
    Vector3f(1, 0, 2),
    Vector3f(1, 0, 1),
    Vector3f(0, 0, 1)
  };
  static const unsigned char chars[43] =
  {
    0x3f, // 0
    0x0c, // 1
    0x76, // 2
    0x5e, // 3
    0x4d, // 4
    0x5b, // 5
    0x7b, // 6
    0x0e, // 7
    0x7f, // 8
    0x5f, // 9
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0x6f, // A
    0x7f, // B
    0x33, // C
    0x3f, // D
    0x73, // E
    0x63, // F
    0x7b, // G
    0x6d, // H
    0x0c, // I
    0x3c, // J
    0x6d, // K
    0x31, // L
    0x2f, // M
    0x6d, // N
    0x3f, // O
    0x67, // P
    0x3f, // Q
    0x6f, // R
    0x5b, // S
    0x0e, // T
    0x3d, // U
    0x3d, // V
    0x3d, // W
    0x6d, // X
    0x4d, // Y
    0x76, // Z
  };
  character = chars[character];
  for(int i = 0; i < 7; ++i)
    if(character & (1 << i))
    {
      Vector3f from = pos - points[i] * size;
      Vector3f to = pos - points[i + 1] * size;
      LINE3D("representation:GameInfo", from.x(), from.y(), from.z(), to.x(), to.y(), to.z(), 2, color);
    }
}

static void print(const std::string& text, const Vector3f& pos, float size, const ColorRGBA& color)
{
  Vector3f charPos = pos;
  for(char character : text)
  {
    drawChar(character - '0', charPos, size, color);
    charPos.x() += size * 1.35f;
  }
}

void GameInfo::draw() const
{
  DEBUG_DRAWING3D("representation:GameInfo", "field")
  {
    float yPosLeftSideline = 3000.f;
    if(Blackboard::getInstance().exists("FieldDimensions"))
      yPosLeftSideline = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]).yPosLeftSideline;
    yPosLeftSideline += 500;

    static std::string gameStates[] =
    {
      "INITIAL",
      "READY",
      "SET",
      "PLAYING",
      "FINISHED",
      "STANDBY"
    };
    print(gameStates[state], Vector3f(static_cast<float>((gameStates[state].size()) * -135.f + 35.f) / 2.f + 100.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    static std::string setPlays[] =
    {
      "",
      "GOAL",
      "PUSHING",
      "CORNER",
      "KICKIN",
      "PENALTY"
    };
    print(setPlays[setPlay], Vector3f(-1635.f, yPosLeftSideline, 500), 100, ColorRGBA::blue);

    const int secsRemaining = std::min(std::abs(static_cast<int>(this->secsRemaining)), 5999);
    const int mins = secsRemaining / 60;
    const int secs = secsRemaining % 60;
    const ColorRGBA color = this->secsRemaining < 0 ? ColorRGBA::red : ColorRGBA::black;
    print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(-350, yPosLeftSideline, 1000), 200, color);
    print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(280, yPosLeftSideline, 1000), 200, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 890, 0, yPosLeftSideline, 910, 3, color);
    LINE3D("representation:GameInfo", 0, yPosLeftSideline, 690, 0, yPosLeftSideline, 710, 3, color);

    if(secondaryTime != 0)
    {
      const int secondaryTime = std::min(std::abs(static_cast<int>(this->secondaryTime)), 5999);
      const int mins = secondaryTime / 60;
      const int secs = secondaryTime % 60;
      const ColorRGBA color = this->secondaryTime < 0 ? ColorRGBA::red : ColorRGBA::blue;
      print(std::to_string(mins / 10) + std::to_string(mins % 10), Vector3f(1285, yPosLeftSideline, 500), 100, color);
      print(std::to_string(secs / 10) + std::to_string(secs % 10), Vector3f(1600, yPosLeftSideline, 500), 100, color);
      LINE3D("representation:GameInfo", 1460, yPosLeftSideline, 445, 1460, yPosLeftSideline, 455, 3, color);
      LINE3D("representation:GameInfo", 1460, yPosLeftSideline, 345, 1460, yPosLeftSideline, 355, 3, color);
    }
  }


  DEBUG_DRAWING3D("representation:GameInfo:scores", "field")
  {
    float yPosLeftSideline = 3000.f;
    if(Blackboard::getInstance().exists("FieldDimensions"))
      yPosLeftSideline = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]).yPosLeftSideline;
    yPosLeftSideline += 500.f;

    for (int iTeam=0; iTeam<2; iTeam++)
    {
      const float x = iTeam == 0 ? -1535.f : 1465.f;
      const int score = std::min(static_cast<int>(teams[iTeam].score), 99);
      // drawDigit(score / 10, Vector3f(x, yPosLeftSideline, 1000), 200, gameInfo.teams[iTeam].fieldPlayerColor);
      // drawDigit(score % 10, Vector3f(x + 270, yPosLeftSideline, 1000), 200, gameInfo.teams[iTeam].fieldPlayerColor);
      print(std::to_string(score / 10) + std::to_string(score % 10), Vector3f(x, yPosLeftSideline, 1000), 200,
            ColorRGBA::fromTeamColor(teams[iTeam].fieldPlayerColor));
    }
  };

  DEBUG_DRAWING3D("representation:GameInfo:player", "robot")
  {
    // for (int iTeam = 0; iTeam < 2; iTeam++)
    // {
    //   for (int iPlayer = 0; iPlayer < playersPerTeam; iPlayer++)
    //   {
    //     if (teams[iTeam].players[iPlayer].penalty != PENALTY_SUBSTITUTE)
    //     {
    //       const int num = GameInfo::toPlayerNumber(iPlayer);
    //       float centerDigit = (num > 1) ? 50.f : 0;
    //       ROTATE3D("representation:GameInfo:player", 0, 0, pi_2);
    //       DRAWDIGIT3D("representation:GameInfo:player", num, Vector3f(centerDigit, 0.f, 500.f), 80, 5, ColorRGBA::green);
    //     }
    //   }
    // }
    const int num = playerNumber;
    float centerDigit = (num > 1) ? 50.f : 0;
    ROTATE3D("representation:GameInfo:player", 0, 0, pi_2);
    DRAWDIGIT3D("representation:GameInfo:player", num, Vector3f(centerDigit, 0.f, 500.f), 80, 5, ColorRGBA::green);
  }

  DEBUG_DRAWING("representation:GameInfo", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    const char* sign = secsRemaining < 0 ? "-" : "";
    const int mins = std::abs(static_cast<int>(secsRemaining)) / 60;
    const int secs = std::abs(static_cast<int>(secsRemaining)) % 60;
    const std::string secsAsString = std::to_string(secs);
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 200, yPosRightFieldBorder + 500,
              (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white,
              "Time remaining: " << sign << mins << ":" << (secs < 10 ? "0" + secsAsString : secsAsString));
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 200, yPosRightFieldBorder + 300,
              (xPosOwnFieldBorder / -5200.f) * 200, ColorRGBA::white, (firstHalf ? "First" : "Second") << " half");
    DRAW_TEXT("representation:GameInfo", xPosOwnFieldBorder + 1700, yPosRightFieldBorder + 300,
              (xPosOwnFieldBorder / -5200.f) * 180, ColorRGBA::white, "State: " << getStateString());
  }

  DEBUG_DRAWING("representation:GameInfo:ourTeam", "drawingOnField")
  {
    float xPosOwnFieldBorder = -5200.f;
    float yPosRightFieldBorder = -3700;
    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      xPosOwnFieldBorder = theFieldDimensions.xPosOwnFieldBorder;
      yPosRightFieldBorder = theFieldDimensions.yPosRightFieldBorder;
    }
    DRAW_TEXT("representation:GameInfo:ourTeam", xPosOwnFieldBorder + 200, yPosRightFieldBorder - 100,
              (xPosOwnFieldBorder / -5200.f) * 140, ColorRGBA::red,
              "Team color: " << TypeRegistry::getEnumName((Settings::TeamColor)ourTeam().fieldPlayerColor));
  }
}

