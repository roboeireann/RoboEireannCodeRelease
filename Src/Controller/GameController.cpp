/**
 * @file Controller/GameController.cpp
 * This file implements a class that simulates a console-based GameController.
 * @author Thomas Röfer
 * @author Arne Hasselbring
 */

#include "GameController.h"
#include "SimulatedRobot.h"

#include "Modules/Communication/TeamMessageHandler2023b/TeamMessageSocketHandler.h"
#include "Tools/Communication/RoboCupGameControlData.h"
// #include "Tools/Communication/SPLStandardMessageBuffer.h"
#include "Platform/SystemCall.h"

#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"

#include "Tools/TextLogging.h"

#include <limits>
#include <algorithm>


DECL_TLOGGER(tlogger, "GameController", TextLogging::INFO);

void GameController::TeamMessageCounter::startLocal(int portIn)
{
  ASSERT(!port);
  port = portIn;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); //keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
  socket.setLoopback(true);
}

void GameController::TeamMessageCounter::receive()
{
  invalidCount = 0;
  messageCount = 0;
  
  if (!port)
    return; // not started yet

  int size;
  // unsigned remoteIp = 0;

  TeamMessageBytes teamMessageBytes;

  do
  {
    size = socket.readLocal(reinterpret_cast<char*>(teamMessageBytes.data), TEAM_MESSAGE_MAX_DATA_SIZE);
    if (size <= 0)
    {
      if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
      {
        TLOGW(tlogger, "receive port {}, problem receiving teammate packet, {}", ErrorStr(errno).str());
        ++invalidCount;
      }
    }
    else if (size > 0)
    {
      ++messageCount;
      TLOGV(tlogger, "receive port {}, valid team comms packet with {} bytes, receivedMessages = {}", 
            port, size, messageCount);
    }
  }
  while(size > 0);
}



const float GameController::footLength = 120.f;
const float GameController::safeDistance = 150.f;
const float GameController::dropHeight = 350.f;

GameController::GameController()
{
  gameInfo.packetNumber = 0;
  gameInfo.playersPerTeam = numPlayersPerTeam;
  gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;
  gameInfo.competitionType = COMPETITION_TYPE_NORMAL;
  gameInfo.gamePhase = GAME_PHASE_NORMAL;
  gameInfo.state = STATE_INITIAL;
  gameInfo.setPlay = SET_PLAY_NONE;
  gameInfo.firstHalf = 1;
  gameInfo.kickingTeam = 1;
  gameInfo.secsRemaining = halfTime;
  gameInfo.secondaryTime = 0;
  for(auto& teamInfo : teamInfos)
    teamInfo.players[gameInfo.playersPerTeam - 1].penalty = PENALTY_SUBSTITUTE; // TODO RV: assumes just one substitute
  resetMessageBudgets();
  // Force reloading of the field dimensions (they cannot be loaded here because the file search path is not available yet).
  fieldDimensions.xPosOwnPenaltyMark = 0.f;
}

void GameController::registerSimulatedRobot(int robotIndex, SimulatedRobot& simulatedRobot)
{
  ASSERT(!robots[robotIndex].simulatedRobot);
  robots[robotIndex].simulatedRobot = &simulatedRobot;
  robots[robotIndex].info.number = RobotNumber::getPlayerNumberFromIndex(robotIndex);
  if (RobotNumber::isSubstituteFromIndex(robotIndex))
    robots[robotIndex].info.penalty = robots[robotIndex].lastPenalty = PENALTY_SUBSTITUTE;
  if (fieldDimensions.xPosOwnPenaltyMark == 0.f)
    fieldDimensions.load();
}


bool GameController::handleStateCommand(const std::string& command, bool fromConsole)
{
  if(command == "initial")
  {
    if(gameInfo.state == STATE_INITIAL)
      return true;

    resetPenaltyTimes();
    timeBeforeCurrentState = 0;
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;

    resetMessageBudgets();

    TLOGI(tlogger, "INITIAL ({})", fromConsole ? "console" : "autoRef");

    return true;
  }
  else if(command == "ready")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      return false;
    if(gameInfo.state == STATE_READY)
      return true;

    if(gameInfo.state == STATE_INITIAL)
      resetPenaltyTimes();
    else if(gameInfo.state == STATE_PLAYING)
      addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_READY;
    if(gameInfo.setPlay != SET_PLAY_PENALTY_KICK)
      gameInfo.setPlay = SET_PLAY_NONE;

    TLOGI(tlogger, "READY ({})", fromConsole ? "console" : "autoRef");

    return true;
  }
  else if(command == "set")
  {
    if(gameInfo.state == STATE_SET)
      return true;

    if(gameInfo.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
      addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
    {
      timeBeforeCurrentState = 0;
      if(gameInfo.state != STATE_INITIAL)
        gameInfo.kickingTeam = gameInfo.kickingTeam == 1 ? 2 : 1;

      for(int i = 0; i < numOfRobots; ++i)
        VERIFY(handleRobotCommand(i, "substitute"));
    }
    gameInfo.state = STATE_SET;
    if(gameInfo.setPlay != SET_PLAY_PENALTY_KICK)
      gameInfo.setPlay = SET_PLAY_NONE;

    TLOGI(tlogger, "SET ({})", fromConsole ? "console" : "autoRef");

    return true;
  }
  else if(command == "playing")
  {
    if(gameInfo.state == STATE_PLAYING)
    {
      gameInfo.setPlay = SET_PLAY_NONE;
      return true;
    }

    if(gameInfo.competitionPhase != COMPETITION_PHASE_PLAYOFF && timeBeforeCurrentState != 0)
      addTimeInCurrentState();

    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_PLAYING;

    TLOGI(tlogger, "PLAYING ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "finished")
  {
    if(gameInfo.state == STATE_FINISHED)
      return true;

    resetPenaltyTimes();
    addTimeInCurrentState();
    timeWhenStateBegan = Time::getCurrentSystemTime();
    gameInfo.state = STATE_FINISHED;
    gameInfo.setPlay = SET_PLAY_NONE;

    TLOGI(tlogger, "FINISHED ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  return false;
}

bool GameController::handleCompetitionPhaseCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "competitionPhasePlayoff")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_PLAYOFF;

    TLOGI(tlogger, "phase = COMPETITION_PHASE_PLAYOFF");
    return true;
  }
  else if(command == "competitionPhaseRoundRobin")
  {
    gameInfo.competitionPhase = COMPETITION_PHASE_ROUNDROBIN;

    TLOGI(tlogger, "phase = COMPETITION_PHASE_ROUNDROBIN");
    return true;
  }
  return false;
}

bool GameController::handleCompetitionTypeCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "competitionTypeNormal")
  {
    gameInfo.competitionType = COMPETITION_TYPE_NORMAL;

    TLOGI(tlogger, "competitionType = COMPETITION_TYPE_NORMAL");
    return true;
  }
  return false;
}

bool GameController::handleGoalCommand(const std::string& command, bool fromConsole)
{
  if(!(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE))
    return false;

  if(command == "goalByFirstTeam")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && gameInfo.kickingTeam != 1)
      return false;

    TLOGI(tlogger, "goal for team 1 ({})", fromConsole ? "console" : "autoRef");

    ++teamInfos[0].score;
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      VERIFY(handleStateCommand("finished"));
    else
    {
      gameInfo.kickingTeam = 2;
      VERIFY(handleStateCommand("ready"));
    }
    return true;
  }
  else if(command == "goalBySecondTeam")
  {
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && gameInfo.kickingTeam != 2)
      return false;

    TLOGI(tlogger, "goal for team 2 ({})", fromConsole ? "console" : "autoRef");

    ++teamInfos[1].score;
    if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      VERIFY(handleStateCommand("finished"));
    else
    {
      gameInfo.kickingTeam = 1;
      VERIFY(handleStateCommand("ready"));
    }
    return true;
  }
  return false;
}

bool GameController::handleSetPlayCommand(const std::string& command, bool fromConsole)
{
  if(!(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT))
    return false;

  if(command == "goalKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "SET_PLAY_GOAL_KICK team 1 ({})", fromConsole ? "console" : "autoRef");

    return true;
  }
  else if(command == "goalKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_GOAL_KICK;
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "SET_PLAY_GOAL_KICK team 2 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "pushingFreeKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "SET_PLAY_PUSHING_FREE_KICK team 1 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "pushingFreeKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PUSHING_FREE_KICK;
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "SET_PLAY_PUSHING_FREE_KICK team 2 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "cornerKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "SET_PLAY_CORNER_KICK team 1 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "cornerKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_CORNER_KICK;
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "SET_PLAY_CORNER_KICK team 2 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "kickInForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "SET_PLAY_KICK_IN team 1 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "kickInForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_KICK_IN;
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "SET_PLAY_KICK_IN team 2 ({})", fromConsole ? "console" : "autoRef");
    return true;
  }
  else if(command == "penaltyKickForFirstTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "SET_PLAY_PENALTY_KICK team 1 ({})", fromConsole ? "console" : "autoRef");
    return handleStateCommand("ready");
  }
  else if(command == "penaltyKickForSecondTeam")
  {
    timeWhenSetPlayBegan = Time::getCurrentSystemTime();
    gameInfo.setPlay = SET_PLAY_PENALTY_KICK;
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "SET_PLAY_PENALTY_KICK team 2 ({})", fromConsole ? "console" : "autoRef");
    return handleStateCommand("ready");
  }
  return false;
}

bool GameController::handleKickOffCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL)
    return false;
  else if(command == "kickOffFirstTeam")
  {
    gameInfo.kickingTeam = 1;

    TLOGI(tlogger, "Kickoff team 1");
    return true;
  }
  else if(command == "kickOffSecondTeam")
  {
    gameInfo.kickingTeam = 2;

    TLOGI(tlogger, "Kickoff team 2");
    return true;
  }
  return false;
}

bool GameController::handleManualPlacementCommand(const std::string& command)
{
  if(gameInfo.state != STATE_INITIAL && gameInfo.state != STATE_SET)
    return false;
  else if(command == "manualPlacementFirstTeam")
  {
    placeGoalie(0);
    if(gameInfo.kickingTeam == 1)
      placeOffensivePlayers(1);
    else
      placeDefensivePlayers(1);

    TLOGI(tlogger, "Manual placement team 1");
    return true;
  }
  else if(command == "manualPlacementSecondTeam")
  {
    placeGoalie(numPlayersPerTeam);
    if(gameInfo.kickingTeam == 2)
      placeOffensivePlayers(numPlayersPerTeam + 1);
    else
      placeDefensivePlayers(numPlayersPerTeam + 1);

    TLOGI(tlogger, "Manual placement team 2");
    return true;
  }
  return false;
}

bool GameController::handleGamePhaseCommand(const std::string& command)
{
  if (command == "gamePenaltyShootout")
  {
    if (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      return true;

    gameInfo.gamePhase = GAME_PHASE_PENALTYSHOOT;
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;
    gameInfo.kickingTeam = 1;
    timeBeforeCurrentState = 0;

    TLOGI(tlogger, "GAME_PHASE_PENALTYSHOOT, INITIAL, kick for team 1");
    return true;
  }
  else if (command == "gameNormal")
  {
    if (gameInfo.gamePhase == GAME_PHASE_NORMAL)
      return true;

    gameInfo.gamePhase = GAME_PHASE_NORMAL;
    gameInfo.state = STATE_INITIAL;
    gameInfo.setPlay = SET_PLAY_NONE;
    timeBeforeCurrentState = 0;

    TLOGI(tlogger, "GAME_PHASE_NORMAL, INITIAL");
    return true;
  }
  return false;
}

bool GameController::handleGlobalCommand(const std::string& command, bool fromConsole)
{
  if(handleStateCommand(command, fromConsole))
    return true;
  else if(handleCompetitionTypeCommand(command))
    return true;
  else if(handleCompetitionPhaseCommand(command))
    return true;
  else if(handleGoalCommand(command, fromConsole))
    return true;
  else if(handleSetPlayCommand(command, fromConsole))
    return true;
  else if(handleKickOffCommand(command))
    return true;
  else if(handleManualPlacementCommand(command))
    return true;
  else if(handleGamePhaseCommand(command))
    return true;

  return false;
}

bool GameController::handleGlobalConsole(In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleGlobalCommand(command, /* fromConsole */ true);
}

bool GameController::handleRobotCommand(int robot, const std::string& command)
{
  Robot& r = robots[robot];
  RoboCup::RobotInfo& tr = teamInfos[robot * 2 / numOfRobots].players[robot % numPlayersPerTeam];
  FOREACH_ENUM(Penalty, i)
    if(command == TypeRegistry::getEnumName(i))
    {
      r.info.penalty = i == manual ? PENALTY_MANUAL : (i == substitute ? PENALTY_SUBSTITUTE : static_cast<uint8_t>(i));
      tr.penalty = r.info.penalty;
      if(i != none)
      {
        r.timeWhenPenalized = Time::getCurrentSystemTime();
        OUTPUT_TEXT(fmt::format("Robot {} penalized with {}", r.info.getRobotAndColourString(), command));
      }
      return true;
    }
  FOREACH_ENUM(RobotInfo::Mode, i)
  {
    if(command == TypeRegistry::getEnumName(i))
    {
      r.info.mode = i;
      return true;
    }
  }
  return false;
}

bool GameController::handleRobotConsole(int robot, In& stream)
{
  SYNC;
  std::string command;
  stream >> command;
  return handleRobotCommand(robot, command);
}

void GameController::placeForPenalty(int robot, float x, float y, float rotation)
{
  Robot& r = robots[robot];
  ASSERT(r.simulatedRobot);
  Vector2f newPos(robot < numPlayersPerTeam ? x : -x, y);
  for(;;)
  {
    int j = 0;
    while(j < numOfRobots && (j == robot || !robots[j].simulatedRobot || (robots[j].lastPose.translation - newPos).squaredNorm() >= sqr(300.f)))
      ++j;
    if(j == numOfRobots)
    {
      r.lastPose = Pose2f(rotation, newPos);
      r.simulatedRobot->moveRobot(Vector3f(newPos.x(), newPos.y(), dropHeight), Vector3f(0.f, 0.f, rotation), true);
      break;
    }
    else
      newPos.x() += newPos.x() < 0.f ? -400.f : 400.f;
  }
}

void GameController::placeGoalie(int robot)
{
  Robot& r = robots[robot];
  if(r.info.penalty != PENALTY_NONE)
    return;
  r.manuallyPlaced = true;
  r.lastPose = robot < numPlayersPerTeam ? Pose2f(-pi, fieldDimensions.xPosOpponentGroundLine - safeDistance, 0.f)
               : Pose2f(0.f, fieldDimensions.xPosOwnGroundLine + safeDistance, 0.f);
}

void GameController::placeFromSet(int robot, int minRobot, const Pose2f* poses)
{
  // For finding a manual placement pose, it is determined which
  // of the positions would be chosen by our teammates.
  bool occupied[numOfFieldPlayers] = {false};
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
    if(i != robot && robots[i].simulatedRobot && robots[i].info.penalty == PENALTY_NONE)
    {
      const Robot& r2 = robots[i];
      float minDistanceSqr = std::numeric_limits<float>::max();
      int bestPoseIndex = 0;
      for(int j = 0; j < numOfFieldPlayers; ++j)
      {
        const Pose2f& pose = poses[j];
        const float distanceSqr = (pose.translation - r2.lastPose.translation).squaredNorm();
        if(!occupied[j] && distanceSqr < minDistanceSqr)
        {
          minDistanceSqr = distanceSqr;
          bestPoseIndex = j;
        }
      }
      occupied[bestPoseIndex] = true;
    }

  // The position that would not be chosen is suitable for this robot.
  int i = 0;
  while(i < numOfFieldPlayers && occupied[i])
    ++i;
  ASSERT(i < numOfFieldPlayers);
  robots[robot].lastPose = poses[i];
}

void GameController::placeOffensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0.f, -fieldDimensions.centerCircleRadius - footLength, 0.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, 0.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2f(-pi, fieldDimensions.centerCircleRadius + footLength, 0.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, 0.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };

  // Move all field players that are not in their own half.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty != PENALTY_NONE)
      continue;
    r.manuallyPlaced = true;
    placeFromSet(i, minRobot, poses[i < numPlayersPerTeam ? 1 : 0]);
  }
}

void GameController::placeDefensivePlayers(int minRobot)
{
  static const Pose2f poses[2][numOfFieldPlayers] =
  {
    {
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, 0.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyArea + safeDistance, 0.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f),
      Pose2f(0.f, fieldDimensions.xPosOwnPenaltyMark, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f)
    },
    {
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, 0.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyArea - safeDistance, 0.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, (fieldDimensions.yPosRightPenaltyArea + fieldDimensions.yPosRightSideline) / 2.f),
      Pose2f(-pi, fieldDimensions.xPosOpponentPenaltyMark, (fieldDimensions.yPosLeftPenaltyArea + fieldDimensions.yPosLeftSideline) / 2.f)
    }
  };

  // Move all field players that are not in their own half or in the center circle.
  for(int i = minRobot; i < minRobot + numOfFieldPlayers; ++i)
  {
    Robot& r = robots[i];
    if(r.info.penalty != PENALTY_NONE)
      continue;
    r.manuallyPlaced = true;
    placeFromSet(i, minRobot, poses[i < numPlayersPerTeam ? 1 : 0]);
  }
}

void GameController::checkIllegalPositionInSet(int robot)
{
  Robot& r = robots[robot];
  if(!r.simulatedRobot || r.info.penalty != PENALTY_NONE)
    return;

  const bool isFirstTeam = robot < numPlayersPerTeam;
  const bool isKickingTeam = gameInfo.kickingTeam == (isFirstTeam ? 1 : 2);
  const bool isGoalkeeper = robot == 0 || robot == numPlayersPerTeam;
  const bool isPenaltyKick = gameInfo.setPlay == SET_PLAY_PENALTY_KICK;
  const bool notOnField = r.lastPose.translation.y() < fieldDimensions.yPosRightSideline - footLength ||
                          r.lastPose.translation.y() > fieldDimensions.yPosLeftSideline + footLength ||
                          r.lastPose.translation.x() < fieldDimensions.xPosOwnGroundLine - footLength ||
                          r.lastPose.translation.x() > fieldDimensions.xPosOpponentGroundLine + footLength;
  const bool inOpponentHalf = isFirstTeam ? (r.lastPose.translation.x() < footLength) : (r.lastPose.translation.x() > -footLength);
  const bool inCenterCircle = r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength);
  const bool notOnOwnGoalLine = std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine)) > footLength ||
                                r.lastPose.translation.y() > fieldDimensions.yPosLeftGoal || r.lastPose.translation.y() < fieldDimensions.yPosRightGoal;
  const bool inOpponentPenaltyArea = r.lastPose.translation.y() > fieldDimensions.yPosRightPenaltyArea - footLength && r.lastPose.translation.y() < fieldDimensions.yPosLeftPenaltyArea + footLength &&
                                     (isFirstTeam ? (r.lastPose.translation.x() < fieldDimensions.xPosOwnPenaltyArea + footLength) : (r.lastPose.translation.x() > fieldDimensions.xPosOpponentPenaltyArea - footLength));
  const bool inOwnPenaltyArea = r.lastPose.translation.y() > fieldDimensions.yPosRightPenaltyArea - footLength && r.lastPose.translation.y() < fieldDimensions.yPosLeftPenaltyArea + footLength &&
                                (isFirstTeam ? (r.lastPose.translation.x() > fieldDimensions.xPosOpponentPenaltyArea - footLength) : (r.lastPose.translation.x() < fieldDimensions.xPosOwnPenaltyArea + footLength));
  const Vector2f opponentPenaltyMark(isFirstTeam ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f);
  const bool onOpponentPenaltyMark = (r.lastPose.translation - opponentPenaltyMark).squaredNorm() < sqr(footLength + fieldDimensions.penaltyMarkSize * 0.5f);
  auto closestToOpponentPenaltyMark = [&]() -> bool
  {
    const float myDistance2 = (r.lastPose.translation - opponentPenaltyMark).squaredNorm();
    const int base = isFirstTeam ? 0 : numPlayersPerTeam;
    for(int i = base; i < base + numPlayersPerTeam; ++i)
    {
      if(i == robot || !robots[i].simulatedRobot || robots[i].info.penalty != PENALTY_NONE)
        continue;
      const float itsDistance2 = (robots[i].lastPose.translation - opponentPenaltyMark).squaredNorm();
      if(itsDistance2 >= sqr(footLength + fieldDimensions.penaltyMarkSize * 0.5f) && (itsDistance2 < myDistance2 || (itsDistance2 == myDistance2 && i < robot)))
        return false;
    }
    return true;
  };

  if(notOnField ||
     (isPenaltyKick ?
      (isKickingTeam ? onOpponentPenaltyMark || (inOpponentPenaltyArea && !closestToOpponentPenaltyMark()) : (isGoalkeeper ? notOnOwnGoalLine : inOwnPenaltyArea)) :
      inOpponentHalf || (!isKickingTeam && inCenterCircle)))
  {
    RoboCup::RobotInfo& tr = teamInfos[robot * 2 / numOfRobots].players[robot % numPlayersPerTeam];
    r.info.penalty = PENALTY_SPL_ILLEGAL_POSITION_IN_SET;
    tr.penalty = r.info.penalty;
    r.timeWhenPenalized = Time::getCurrentSystemTime();
  }
}

void GameController::executePlacement()
{
  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];
    if(r.manuallyPlaced)
      r.simulatedRobot->moveRobot(Vector3f(r.lastPose.translation.x(), r.lastPose.translation.y(), dropHeight),
                                  Vector3f(0.f, 0.f, r.lastPose.rotation), true);
    r.manuallyPlaced = false;
  }
}

void GameController::updateAndReferee()
{
  SYNC;

  if(lastState != STATE_SET && gameInfo.state == STATE_SET)
  {
    if(automatic & bit(penalizeIllegalPositionInSet) && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
      for(int i = 0; i < numOfRobots; ++i)
        checkIllegalPositionInSet(i);

    if(automatic & bit(placeBall))
    {
      if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT || gameInfo.setPlay == SET_PLAY_PENALTY_KICK)
        SimulatedRobot::moveBall(Vector3f(gameInfo.kickingTeam == 1 ? fieldDimensions.xPosOwnPenaltyMark : fieldDimensions.xPosOpponentPenaltyMark, 0.f, 50.f), true);
      else
        SimulatedRobot::moveBall(Vector3f(0.f, 0.f, 50.f), true);
    }
  }
  lastState = gameInfo.state;

  executePlacement();

  // Prepare numbers for illegal positioning.
  if(automatic & bit(penalizeIllegalPosition))
  {
    for(int i = 0; i < numOfRobots; ++i)
    {
      Robot& r = robots[i];
      if(r.info.penalty == PENALTY_NONE && r.simulatedRobot)
      {
        const bool isFirstTeam = i < numPlayersPerTeam;

        // inside penalty areas?
        float yMargin = std::min(r.lastPose.translation.y() - fieldDimensions.yPosRightPenaltyArea + footLength,
                                       fieldDimensions.yPosLeftPenaltyArea + footLength - r.lastPose.translation.y());
        float xMarginOwn =
            isFirstTeam ? (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength)
                        : (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x());
        float xMarginOpponent =
            isFirstTeam ? (fieldDimensions.xPosOwnPenaltyArea + footLength - r.lastPose.translation.x())
                        : (r.lastPose.translation.x() - fieldDimensions.xPosOpponentPenaltyArea + footLength);
        r.ownPenaltyAreaMargin = std::min(yMargin, xMarginOwn);
        r.opponentPenaltyAreaMargin = std::min(yMargin, xMarginOpponent);

        // inside own goal area?
        // inside => margin is positive, outside => margin is negative
        yMargin = std::min(r.lastPose.translation.y() - fieldDimensions.yPosRightGoalArea + footLength,
                                       fieldDimensions.yPosLeftGoalArea + footLength - r.lastPose.translation.y());
        xMarginOwn =
            isFirstTeam ? (r.lastPose.translation.x() - fieldDimensions.xPosOpponentGoalArea + footLength)
                        : (fieldDimensions.xPosOwnGoalArea + footLength - r.lastPose.translation.x());
        r.ownGoalAreaMargin = std::min(yMargin, xMarginOwn);
      }
      else
        r.ownGoalAreaMargin = -1.f;
    }
  }

  for(int i = 0; i < numOfRobots; ++i)
  {
    Robot& r = robots[i];

    if(r.info.penalty == PENALTY_NONE && r.simulatedRobot)
    {
      auto inCenterCircleBeforeBallIsInPlay = [&]
      {
        return gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_NONE &&
               lastBallContactTime < timeWhenStateBegan && Time::getTimeSince(timeWhenStateBegan) < 10000 &&
               gameInfo.kickingTeam != (i < numPlayersPerTeam ? 1 : 2) &&
               r.lastPose.translation.squaredNorm() < sqr(fieldDimensions.centerCircleRadius + footLength);
      };
      auto inPenaltyAreaDuringPenaltyKick = [&]
      {
        if (gameInfo.state != STATE_PLAYING || gameInfo.setPlay != SET_PLAY_PENALTY_KICK)
          return false;
        const bool isFirstTeam = i < numPlayersPerTeam;
        const bool isKickingTeam = gameInfo.kickingTeam == (isFirstTeam ? 1 : 2);
        // Outside the penalty area is legal.
        if((isKickingTeam ? r.opponentPenaltyAreaMargin : r.ownPenaltyAreaMargin) < 0.f)
          return false;
        // The goalkeeper is only allowed to stand on the goal line.
        // (but if it's outside the penalty area, that's okay, too, because that will happen when it's unpenalized during a penalty kick).
        if(!isKickingTeam && i == (isFirstTeam ? 0 : numPlayersPerTeam) &&
           std::abs(r.lastPose.translation.x() - (isFirstTeam ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine)) < footLength &&
           r.lastPose.translation.y() < fieldDimensions.yPosLeftGoal && r.lastPose.translation.y() > fieldDimensions.yPosRightGoal)
          return false;
        // No other defender may be in there.
        if(!isKickingTeam)
          return true;
        // The player of the kicking team which is deepest inside the opponent's penalty area is considered to be the taker which is allowed to be there.
        // All others are not.
        for(int j = isFirstTeam ? 0 : numPlayersPerTeam; j < (isFirstTeam ? numPlayersPerTeam : numOfRobots); ++j)
          if(i != j && (robots[j].opponentPenaltyAreaMargin > r.opponentPenaltyAreaMargin || (robots[j].opponentPenaltyAreaMargin == r.opponentPenaltyAreaMargin && j < i)))
            return true;
        return false;
      };
      // auto inFullPenaltyArea = [&] // 2022 and before
      // {
      //   const bool isFirstTeam = i < numPlayersPerTeam;
      //   // Count the number of players which are deeper inside the respective penalty area.
      //   // If more than 3 players are, this player must be illegal.
      //   if(r.ownPenaltyAreaMargin >= 0.f)
      //   {
      //     int counter = 0;
      //     for(int j = isFirstTeam ? 0 : numPlayersPerTeam; j < (isFirstTeam ? numPlayersPerTeam : numOfRobots); ++j)
      //       if(i != j && (robots[j].ownPenaltyAreaMargin > r.ownPenaltyAreaMargin || (robots[j].ownPenaltyAreaMargin == r.ownPenaltyAreaMargin && j < i)))
      //         ++counter;
      //     if(counter >= 3)
      //       return true;
      //   }
      //   if(r.opponentPenaltyAreaMargin >= 0.f)
      //   {
      //     int counter = 0;
      //     for(int j = isFirstTeam ? 0 : numPlayersPerTeam; j < (isFirstTeam ? numPlayersPerTeam : numOfRobots); ++j)
      //       if(i != j && (robots[j].opponentPenaltyAreaMargin > r.opponentPenaltyAreaMargin || (robots[j].opponentPenaltyAreaMargin == r.opponentPenaltyAreaMargin && j < i)))
      //         ++counter;
      //     if(counter >= 3)
      //       return true;
      //   }
      //   return false;
      // };
      auto inOwnGoalArea = [&] // 2023 on
      {
        const bool isFirstTeam = i < numPlayersPerTeam;
        // Count the number of players which are deeper inside the respective goal area.
        // If more than 3 players are, this player must be illegal.
        if(r.ownGoalAreaMargin >= 0.f)
        {
          int counter = 0;
          for(int j = isFirstTeam ? 0 : numPlayersPerTeam; j < (isFirstTeam ? numPlayersPerTeam : numOfRobots); ++j)
            if(i != j && (robots[j].ownGoalAreaMargin > r.ownGoalAreaMargin || (robots[j].ownGoalAreaMargin == r.ownGoalAreaMargin && j < i)))
              ++counter;
          if(counter >= 3)
            return true;
        }
        return false;
      };
      auto inFreeKickArea = [&]
      {
        const bool isFirstTeam = i < numPlayersPerTeam;
        if(gameInfo.state != STATE_PLAYING || gameInfo.setPlay == SET_PLAY_NONE || gameInfo.setPlay == SET_PLAY_PENALTY_KICK ||
           gameInfo.kickingTeam == (isFirstTeam ? 1 : 2))
          return false;
        // A goalkeeper within its own penalty area is allowed to approach the ball.
        // TODO: It is still not allowed to touch the ball.
        if(i == (isFirstTeam ? 0 : numPlayersPerTeam) && r.ownPenaltyAreaMargin >= 0.f)
          return false;

        // For now, just penalize anyone who is within 75cm after 10s.
        // TODO: This does not consider:
        //   - Players that clearly violate the rule by walking to or playing the ball before 10s
        //   - Players that are inside the circle after 10s because they are fallen or pushed
        Vector2f ballPos;
        SimulatedRobot::getAbsoluteBallPosition(ballPos);
        return Time::getTimeSince(timeWhenSetPlayBegan) > 10000 && (r.lastPose.translation - ballPos).squaredNorm() < sqr(750.f);
      };

      if (automatic & bit(penalizeLeavingTheField) && !fieldDimensions.isInsideCarpet(r.lastPose.translation))
        VERIFY(handleRobotCommand(i, "leavingTheField"));
      else if (automatic & bit(penalizeIllegalPosition) &&
               (inCenterCircleBeforeBallIsInPlay() || inPenaltyAreaDuringPenaltyKick() || inOwnGoalArea() ||
                inFreeKickArea()))
      {
        TLOGI(tlogger, "illegalPosition: inCircleBeforePlay {}, inPenAreaDuringPenKick {}, inOwnGoalArea {}, inFreeArea {}",
              inCenterCircleBeforeBallIsInPlay(), inPenaltyAreaDuringPenaltyKick(), inOwnGoalArea(), inFreeKickArea());
        VERIFY(handleRobotCommand(i, "illegalPosition"));
      }
    }

    if(automatic & bit(placePlayers) && r.info.penalty != PENALTY_NONE && r.lastPenalty == PENALTY_NONE && r.simulatedRobot)
    {
      placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                      fieldDimensions.yPosRightFieldBorder + 100.f, -pi_2);
    }

    if(r.info.penalty != PENALTY_NONE)
    {
      r.info.secsTillUnpenalised = static_cast<uint8_t>(std::max<int>((r.info.penalty == PENALTY_SPL_ILLEGAL_POSITION_IN_SET ? 15 : 45) - Time::getTimeSince(r.timeWhenPenalized) / 1000, 0));
      RoboCup::RobotInfo& tr = teamInfos[i * 2 / numOfRobots].players[i % (numPlayersPerTeam)];
      tr.secsTillUnpenalised = r.info.secsTillUnpenalised;

      if(automatic & bit(unpenalize) && r.info.secsTillUnpenalised <= 0 && r.info.penalty != PENALTY_MANUAL && r.info.penalty != PENALTY_SUBSTITUTE)
      {
        r.info.penalty = PENALTY_NONE;
        tr.penalty = PENALTY_NONE;
      }
    }

    if(automatic & bit(placePlayers) && r.info.penalty == PENALTY_NONE && r.lastPenalty != PENALTY_NONE && r.simulatedRobot)
    {
      if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      {
        Pose2f newPose((i < numPlayersPerTeam) ? pi : 0.f, 0.f, 0.f);
        if(gameInfo.kickingTeam == (i < numPlayersPerTeam ? 1 : 2))
          newPose.translate(fieldDimensions.xPosPenaltyStrikerStartPosition, 0.f);
        else
          newPose.translate(fieldDimensions.xPosOwnGroundLine, 0.f);
        r.simulatedRobot->moveRobot(Vector3f(newPose.translation.x(), newPose.translation.y(), dropHeight), Vector3f(0.f, 0.f, newPose.rotation), true);
      }
      else
      {
        Vector2f ballPos;
        r.simulatedRobot->getAbsoluteBallPosition(ballPos);
        placeForPenalty(i, fieldDimensions.xPosOpponentPenaltyMark,
                        ballPos.y() >= 0.f ? fieldDimensions.yPosRightSideline : fieldDimensions.yPosLeftSideline,
                        ballPos.y() >= 0.f ? pi_2 : -pi_2);
      }
    }

    if(automatic & bit(clearBall) && r.simulatedRobot)
    {
      Vector2f ballPos;
      r.simulatedRobot->getAbsoluteBallPosition(ballPos);
      if((r.lastPose * Vector2f(50.f, 0.f) - ballPos).norm() > 50.f)
        r.timeWhenBallNotStuckBetweenLegs = Time::getCurrentSystemTime();
      else if(r.timeWhenBallNotStuckBetweenLegs && Time::getTimeSince(r.timeWhenBallNotStuckBetweenLegs) > 500)
        SimulatedRobot::moveBall((Vector3f() << r.lastPose * Vector2f(150.f, 0.f), 50.f).finished(), true);
    }

    r.lastPenalty = r.info.penalty;
  }

  switch(gameInfo.state)
  {
    case STATE_READY:
      if(Time::getTimeSince(timeWhenStateBegan) < 2000)
        timeWhenLastRobotMoved = 0;
      if(Time::getTimeSince(timeWhenStateBegan) >= (gameInfo.setPlay == SET_PLAY_PENALTY_KICK ? penaltyKickReadyTime : readyTime) * 1000 ||
         (automatic & bit(switchToSet) && timeWhenLastRobotMoved && Time::getTimeSince(timeWhenLastRobotMoved) > 2000))
        handleStateCommand("set");
      break;

    case STATE_SET:
      if(automatic & bit(switchToPlaying) && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && Time::getTimeSince(timeWhenStateBegan) >= 5000)
        handleStateCommand("playing");
      break;

    case STATE_PLAYING:
      if(gameInfo.setPlay != SET_PLAY_NONE && (gameInfo.setPlay != SET_PLAY_PENALTY_KICK ?
                                               Time::getTimeSince(timeWhenSetPlayBegan) >= freeKickTime * 1000 :
                                               Time::getTimeSince(timeWhenStateBegan) >= penaltyShotTime * 1000))
        gameInfo.setPlay = SET_PLAY_NONE;

      if(automatic & bit(freeKickComplete) && gameInfo.setPlay != SET_PLAY_NONE &&
         lastBallContactTime > timeWhenSetPlayBegan + 500 && (gameInfo.kickingTeam == 1) != (lastBallContactPose.rotation == 0.f))
        VERIFY(handleStateCommand("playing"));

      if(automatic & bit(ballOut))
      {
        const auto ballOutType = updateBall();
        if(gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && ballOutType != (gameInfo.kickingTeam == 1 ? goalByFirstTeam : goalBySecondTeam) && ballOutType != notOut)
          VERIFY(handleStateCommand("finished"));
        else if(ballOutType != notOut)
        {
          // It is possible that setPlay is not NONE here because the autoreferee may
          // not have detected a completed free kick or an opponent (which actually is
          // not allowed to do so) touched the ball.
          gameInfo.setPlay = SET_PLAY_NONE;
          switch(ballOutType)
          {
            case goalByFirstTeam:
              VERIFY(handleGoalCommand("goalByFirstTeam"));
              break;
            case goalBySecondTeam:
              VERIFY(handleGoalCommand("goalBySecondTeam"));
              break;
            case outByFirstTeam:
              VERIFY(handleSetPlayCommand("kickInForSecondTeam"));
              break;
            case outBySecondTeam:
              VERIFY(handleSetPlayCommand("kickInForFirstTeam"));
              break;
            case ownGoalOutByFirstTeam:
              VERIFY(handleSetPlayCommand("cornerKickForSecondTeam"));
              break;
            case ownGoalOutBySecondTeam:
              VERIFY(handleSetPlayCommand("cornerKickForFirstTeam"));
              break;
            case opponentGoalOutByFirstTeam:
              VERIFY(handleSetPlayCommand("goalKickForSecondTeam"));
              break;
            case opponentGoalOutBySecondTeam:
              VERIFY(handleSetPlayCommand("goalKickForFirstTeam"));
              break;
            default:
              break;
          }
        }
      }
  }

  // receive SPLStandardMessages from robots to count and check validity
  for (int team = 0; team < 2; team++)
  {
    teamMessageCounters[team].receive();

    if (gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && 
        gameInfo.state != STATE_INITIAL &&
        gameInfo.state != STATE_FINISHED)
    {
      messageBudgets[team] -= teamMessageCounters[team].messageCount;
      invalidCounts[team] += teamMessageCounters[team].invalidCount;

      teamInfos[team].messageBudget = static_cast<uint16_t>(std::max(0, messageBudgets[team]));

      if (teamMessageCounters[team].messageCount || teamMessageCounters[team].invalidCount)
          TLOGD(tlogger, "team {} received new {} (total {}), new invalid {} (total {}), remaining budget {}", team + 1,
                teamMessageCounters[team].messageCount, initialMessageBudget - messageBudgets[team],
                teamMessageCounters[team].invalidCount, invalidCounts[team], teamInfos[team].messageBudget);
    }
  }

  // GameController packets are sent approx 2 per second (i.e. ~500ms interval)
  if (Time::getTimeSince(timeLastGCPacket) > 500)
  {
    gameInfo.packetNumber++; // generate a new packet
    timeLastGCPacket = Time::getCurrentSystemTime();

    TLOGD(tlogger, "updateAndReferee: new GC packet {}", gameInfo.packetNumber);
  }

}

void GameController::addTimeInCurrentState()
{
  timeBeforeCurrentState += Time::getCurrentSystemTime() - timeWhenStateBegan;
}

void GameController::resetPenaltyTimes()
{
  for(auto& robot : robots)
    robot.timeWhenPenalized = 0;
}

void GameController::resetMessageBudgets()
{
  for (int team = 0; team < 2; team++)
  {
    messageBudgets[team] = initialMessageBudget;
    invalidCounts[team] = 0;

    teamInfos[team].messageBudget = static_cast<uint16_t>(initialMessageBudget);
  }
}

GameController::BallOut GameController::updateBall()
{
  BallOut result = notOut;
  Vector2f ballPos;
  SimulatedRobot::getAbsoluteBallPosition(ballPos);
  Vector2f ballInnerEdge(ballPos.x() - sgn(ballPos.x()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f),
                         ballPos.y() - sgn(ballPos.y()) * (ballSpecification.radius + fieldDimensions.fieldLinesWidth / 2.f));
  if(!fieldDimensions.isInsideField(ballInnerEdge))
  {
    if(std::abs(ballPos.y()) < fieldDimensions.yPosLeftGoal) // goal
      result = ballPos.x() > 0.f ? goalBySecondTeam : goalByFirstTeam;
    else
    {
      if(std::abs(ballPos.x()) > fieldDimensions.xPosOpponentGroundLine)
      {
        if((ballPos.x() > 0.f) == (lastBallContactPose.rotation == 0.f)) // goal kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGoalArea : fieldDimensions.xPosOwnGoalArea;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightGoalArea;
          else
            ballPos.y() = fieldDimensions.yPosLeftGoalArea;

          result = lastBallContactPose.rotation == 0.f ? opponentGoalOutBySecondTeam : opponentGoalOutByFirstTeam;
        }
        else // corner kick
        {
          ballPos.x() = ballPos.x() > 0.f ? fieldDimensions.xPosOpponentGroundLine : fieldDimensions.xPosOwnGroundLine;

          if(ballPos.y() < 0.f)
            ballPos.y() = fieldDimensions.yPosRightSideline;
          else
            ballPos.y() = fieldDimensions.yPosLeftSideline;

          result = lastBallContactPose.rotation == 0.f ? ownGoalOutBySecondTeam : ownGoalOutByFirstTeam;
        }
      }
      else // kick in
      {
        float x = ballPos.x();
        if(x < fieldDimensions.xPosOwnGroundLine)
          x = fieldDimensions.xPosOwnGroundLine; // clip
        else if(x > fieldDimensions.xPosOpponentGroundLine)
          x = fieldDimensions.xPosOpponentGroundLine; // clip
        ballPos.x() = x;

        if(ballPos.y() < 0.f)
          ballPos.y() = fieldDimensions.yPosRightSideline;
        else
          ballPos.y() = fieldDimensions.yPosLeftSideline;

        result = lastBallContactPose.rotation == 0.f ? outBySecondTeam : outByFirstTeam;
      }

      SimulatedRobot::moveBall(Vector3f(ballPos.x(), ballPos.y(), 100.f), true);
    }
  }
  return result;
}

void GameController::setLastBallContactRobot(SimRobot::Object* robot)
{
  lastBallContactPose = Pose2f(SimulatedRobot::isFirstTeam(robot) ? pi : 0.f, SimulatedRobot::getPosition(robot));
  lastBallContactTime = Time::getCurrentSystemTime();
}

bool GameController::writeGameInfo(Out& stream, uint8_t& lastGCPacketNumber)
{
  SYNC;

  // check if there is a new packet and return immediately if there isn't
  if (gameInfo.packetNumber == lastGCPacketNumber)
    return false;
  else
    lastGCPacketNumber = gameInfo.packetNumber;

  const int duration = gameInfo.gamePhase == GAME_PHASE_NORMAL ? halfTime : penaltyShotTime;
  const int timePlayed = gameInfo.state == STATE_INITIAL
                         || ((gameInfo.state == STATE_READY || gameInfo.state == STATE_SET)
                             && (gameInfo.competitionPhase == COMPETITION_PHASE_PLAYOFF || timeBeforeCurrentState == 0))
                         || gameInfo.state == STATE_FINISHED
                         ? timeBeforeCurrentState / 1000
                         : Time::getTimeSince(timeWhenStateBegan - timeBeforeCurrentState) / 1000;
  gameInfo.secsRemaining = static_cast<int16_t>(duration - timePlayed);

  if(gameInfo.state == STATE_READY)
    gameInfo.secondaryTime = static_cast<int16_t>((gameInfo.setPlay == SET_PLAY_PENALTY_KICK ? penaltyKickReadyTime : readyTime) - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.setPlay == SET_PLAY_PENALTY_KICK)
    gameInfo.secondaryTime = static_cast<int16_t>(penaltyShotTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.setPlay != SET_PLAY_NONE)
    gameInfo.secondaryTime = static_cast<int16_t>(freeKickTime - Time::getTimeSince(timeWhenSetPlayBegan) / 1000);
  else if(gameInfo.state == STATE_PLAYING && gameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && kickOffTime >= Time::getTimeSince(timeWhenStateBegan) / 1000)
    gameInfo.secondaryTime = static_cast<int16_t>(kickOffTime - Time::getTimeSince(timeWhenStateBegan) / 1000);
  else
    gameInfo.secondaryTime = 0;

  gameInfo.timeLastPacketReceived = Time::getCurrentSystemTime();

  TLOGV(tlogger, "writeGameInfo packetNumber {}, timeLastPacketReceived {}", gameInfo.packetNumber,
        gameInfo.timeLastPacketReceived);

  stream << gameInfo;

  return true;
}

void GameController::writeOwnTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[robot * 2 / numOfRobots];
}

void GameController::writeOpponentTeamInfo(int robot, Out& stream)
{
  SYNC;
  stream << teamInfos[1 - robot * 2 / numOfRobots];
}

void GameController::writeRobotInfo(int robot, Out& stream)
{
  SYNC;
  Robot& r = robots[robot];
  Pose2f pose;
  ASSERT(r.simulatedRobot);
  r.simulatedRobot->getRobotPose(pose);
  if(robot < numPlayersPerTeam)
    pose = Pose2f(pi) + pose;
  if((pose.translation - r.lastPose.translation).squaredNorm() > sqr(5.f) ||
     std::abs(Angle::normalize(pose.rotation - r.lastPose.rotation)) > 0.05f)
  {
    timeWhenLastRobotMoved = Time::getCurrentSystemTime();
    r.lastPose = pose;
  }
  stream << r.info;
}

void GameController::addCompletion(std::set<std::string>& completion) const
{
  static const char* commands[] =
  {
    "initial",
    "ready",
    "set",
    "playing",
    "finished",
    "competitionTypeNormal",
    "competitionPhasePlayoff",
    "competitionPhaseRoundRobin",
    "manualPlacementFirstTeam",
    "manualPlacementSecondTeam",
    "goalByFirstTeam",
    "goalBySecondTeam",
    "goalKickForFirstTeam",
    "goalKickForSecondTeam",
    "pushingFreeKickForFirstTeam",
    "pushingFreeKickForSecondTeam",
    "cornerKickForFirstTeam",
    "cornerKickForSecondTeam",
    "kickInForFirstTeam",
    "kickInForSecondTeam",
    "penaltyKickForFirstTeam",
    "penaltyKickForSecondTeam",
    "kickOffFirstTeam",
    "kickOffSecondTeam",
    "gamePenaltyShootout",
    "gameNormal"
  };
  const int num = sizeof(commands) / sizeof(commands[0]);
  for(int i = 0; i < num; ++i)
    completion.insert(std::string("gc ") + commands[i]);
  FOREACH_ENUM(Penalty, i)
    completion.insert(std::string("pr ") + TypeRegistry::getEnumName(i));
  FOREACH_ENUM(RobotInfo::Mode, i)
    completion.insert(std::string("pr ") + TypeRegistry::getEnumName(i));
}

void GameController::setTeamInfos(Settings::TeamColor fieldPlayerColor1, Settings::TeamColor goalkeeperColor1,
                                  Settings::TeamColor fieldPlayerColor2, Settings::TeamColor goalkeeperColor2,
                                  int portTeam1, int portTeam2)
{
  teamInfos[0].teamNumber = 1;
  teamInfos[0].fieldPlayerColor = fieldPlayerColor1;
  teamInfos[0].goalkeeperColor = goalkeeperColor1;
  teamInfos[0].goalkeeper = 1;

  teamInfos[1].teamNumber = 2;
  teamInfos[1].fieldPlayerColor = fieldPlayerColor2;
  teamInfos[1].goalkeeperColor = goalkeeperColor2;
  teamInfos[1].goalkeeper = 1;

  resetMessageBudgets();

  teamMessageCounters[0].startLocal(portTeam1);
  teamMessageCounters[1].startLocal(portTeam2);  
}

