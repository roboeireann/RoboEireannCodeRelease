/**
 * @file GameDataProvider.cpp
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#include "GameDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/TextLogging.h"

#ifdef TARGET_ROBOT
#include <arpa/inet.h>
#include <netinet/in.h>
#endif
#include <cstring>

MAKE_MODULE(GameDataProvider, communication);

DECL_TLOGGER(tlogger, "GameDataProvider", TextLogging::INFO);

GameDataProvider::GameDataProvider()
{
  // Initialize GameController packet
  resetGameCtrlData();

  // Initialize socket
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", GAMECONTROLLER_DATA_PORT));
}

void GameDataProvider::update(RobotInfo& theRobotInfo)
{
  if(!(gameCtrlData.gamePhase != GAME_PHASE_PENALTYSHOOT && gameCtrlData.state == STATE_FINISHED))
    whenStateNotFinished = theFrameInfo.time;

  ignoreChestButton = false;
  switch(mode)
  {
    case RobotInfo::unstiff:
      if(theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      {
        mode = RobotInfo::active;
        ignoreChestButton = true;
      }
      break;
    case RobotInfo::active:
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theFrameInfo.getTimeSince(whenStateNotFinished) > unstiffFinishedDuration)
      {
        resetGameCtrlData();
        mode = RobotInfo::unstiff;
      }
      else if(gameCtrlData.state == STATE_INITIAL &&
              gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1].players[Global::getSettings().playerNumber - 1].penalty == PENALTY_NONE &&
              theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
              theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
      {
        resetGameCtrlData();
        gameCtrlData.state = STATE_PLAYING;
        mode = RobotInfo::calibration;
      }
      break;
    case RobotInfo::calibration:
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theBehaviorStatus.activity == BehaviorStatus::calibrationFinished)
      {
        resetGameCtrlData();
        mode = RobotInfo::unstiff;
      }
      break;
  }

  receive(mode == RobotInfo::active);


  if(theFrameInfo.getTimeSince(whenPacketWasReceived) >= gameControllerTimeout)
    handleButtons();
  else if(theFrameInfo.getTimeSince(whenPacketWasSent) >= aliveDelay && sendAliveMessage())
    whenPacketWasSent = theFrameInfo.time;

  const RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  static_cast<RoboCup::RobotInfo&>(theRobotInfo) = team.players[Global::getSettings().playerNumber - 1];
  theRobotInfo.number = Global::getSettings().playerNumber;
  theRobotInfo.mode = mode;

  DEBUG_RESPONSE_ONCE("module:GameDataProvider:robotInfo")
    OUTPUT(idRobotInfo, bin, theRobotInfo);
}

void GameDataProvider::update(OwnTeamInfo& theOwnTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(theOwnTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
}

void GameDataProvider::update(OpponentTeamInfo& theOpponentTeamInfo)
{
  static_cast<RoboCup::TeamInfo&>(theOpponentTeamInfo) = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 1 : 0];
}

void GameDataProvider::update(RawGameInfo& theRawGameInfo)
{
  std::memcpy(static_cast<RoboCup::RoboCupGameControlData*>(&theRawGameInfo), &gameCtrlData, sizeof(gameCtrlData));
  theRawGameInfo.timeLastPacketReceived = whenGameCtrlDataWasSet;
}

void GameDataProvider::receive(bool setGameCtrlData)
{
  int size;
  RoboCup::RoboCupGameControlData buffer;
  unsigned from;
  while((size = socket.read(reinterpret_cast<char*>(&buffer), sizeof(buffer), from)) > 0)
  {
    if(size == sizeof(buffer) &&
       !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
       buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
       (buffer.teams[0].teamNumber == Global::getSettings().teamNumber ||
        buffer.teams[1].teamNumber == Global::getSettings().teamNumber))
    {
      if(setGameCtrlData)
      {
        gameCtrlData = buffer;
        whenGameCtrlDataWasSet = Time::getCurrentSystemTime(); // more accurate than theFrameInfo.time?;
        TLOGD(tlogger, "receive time {} ({:#032b})", whenGameCtrlDataWasSet, whenGameCtrlDataWasSet);
      }
#ifdef TARGET_ROBOT
      unsigned ip = htonl(from);
      socket.setTarget(inet_ntoa(reinterpret_cast<in_addr&>(ip)), GAMECONTROLLER_RETURN_PORT);
#endif
      whenPacketWasReceived = theFrameInfo.time;
    }
  }
}

bool GameDataProvider::sendAliveMessage()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  
  returnPacket.teamNum = static_cast<std::uint8_t>(Global::getSettings().teamNumber);
  returnPacket.playerNum = static_cast<std::uint8_t>(Global::getSettings().playerNumber);

  bool fallen = theFallDownState.state != FallDownState::upright &&
                theFallDownState.state != FallDownState::staggering &&
                theFallDownState.state != FallDownState::squatting;
  returnPacket.fallen = static_cast<std::uint8_t>(fallen);

  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;

  returnPacket.ballAge = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;

  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();

  return socket.write(reinterpret_cast<char*>(&returnPacket), sizeof(returnPacket));
}

void GameDataProvider::handleButtons()
{
  RoboCup::TeamInfo& team = gameCtrlData.teams[gameCtrlData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1];
  RoboCup::RobotInfo& player = team.players[Global::getSettings().playerNumber - 1];

  if(mode == RobotInfo::active && !ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
  {
    if(player.penalty == PENALTY_NONE)
      player.penalty = PENALTY_MANUAL;
    else
    {
      player.penalty = PENALTY_NONE;
      gameCtrlData.state = STATE_PLAYING;
    }
  }

  if (gameCtrlData.state == STATE_INITIAL && player.penalty == PENALTY_NONE)
  {
    if (theEnhancedKeyStates.hitStreak[KeyStates::lFootLeft] == 1)
    {
      team.fieldPlayerColor = (team.fieldPlayerColor + 1) % (TEAM_GRAY + 1); // cycle between TEAM_BLUE .. TEAM_GRAY

      switch (team.fieldPlayerColor)
      {
        case TEAM_BLUE: SystemCall::say("Team blue"); break;
        case TEAM_RED: SystemCall::say("Team red"); break;
        case TEAM_YELLOW: SystemCall::say("Team yellow"); break;
        case TEAM_BLACK: SystemCall::say("Team black"); break;
        case TEAM_WHITE: SystemCall::say("Team white"); break;
        case TEAM_GREEN: SystemCall::say("Team green"); break;
        case TEAM_ORANGE: SystemCall::say("Team orange"); break;
        case TEAM_PURPLE: SystemCall::say("Team purple"); break;
        case TEAM_BROWN: SystemCall::say("Team brown"); break;
        case TEAM_GRAY: SystemCall::say("Team gray"); break;
      }
    }

    if(theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
    {
      if(gameCtrlData.gamePhase == GAME_PHASE_NORMAL)
      {
        gameCtrlData.gamePhase = GAME_PHASE_PENALTYSHOOT;
        gameCtrlData.kickingTeam = team.teamNumber;
        SystemCall::say("Penalty striker");
      }
      else if(gameCtrlData.kickingTeam == team.teamNumber)
      {
        gameCtrlData.kickingTeam = 0;
        SystemCall::say("Penalty keeper");
      }
      else
        gameCtrlData.gamePhase = GAME_PHASE_NORMAL;
    }
  }
}

void GameDataProvider::resetGameCtrlData()
{
  std::memset(&gameCtrlData, 0, sizeof(gameCtrlData));
  gameCtrlData.teams[0].teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  gameCtrlData.teams[0].fieldPlayerColor = static_cast<uint8_t>(Global::getSettings().fieldPlayerColor);
  gameCtrlData.teams[0].goalkeeperColor = static_cast<uint8_t>(Global::getSettings().goalkeeperColor);
  gameCtrlData.teams[0].goalkeeper = 1; // default to player 1 for now (TODO: maybe in the settings later)

  gameCtrlData.teams[1].fieldPlayerColor = gameCtrlData.teams[0].fieldPlayerColor ^ 1; // we don't know better
  gameCtrlData.teams[1].goalkeeperColor = gameCtrlData.teams[0].goalkeeperColor ^ 1; // we don't know better
  
  gameCtrlData.playersPerTeam = static_cast<uint8_t>(Global::getSettings().playerNumber); // we don't know better
  gameCtrlData.firstHalf = 1;
  whenGameCtrlDataWasSet = 0;
}
