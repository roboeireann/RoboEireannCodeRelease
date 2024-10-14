/**
 * @file ExtendedGameInfoProvider.cpp
 *
 * This file declares a module that provides extended information about the game state.
 *
 * @author Andreas Stolpmann
 * @author Tim Laue
 * @author Jesse Richter-Klug
 * @author Arne Hasselbring
 */

#include "ExtendedGameInfoProvider.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(ExtendedGameInfoProvider, infrastructure);

void ExtendedGameInfoProvider::update(ExtendedGameInfo& extendedGameInfo)
{
  if(theReceivedGameControlData.state != STATE_PLAYING)
    extendedGameInfo.didNotHearWhistleThisTime = false;

  if(theReceivedGameControlData.state != rawGameStateLastFrame
     && theReceivedGameControlData.state == STATE_PLAYING
     && gameStateLastFrame != STATE_PLAYING
     && theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT
     && SystemCall::getMode() != SystemCall::simulatedRobot)
  {
    extendedGameInfo.didNotHearWhistleThisTime = true;
    timeWhenLastPlayingStarted = theFrameInfo.time - timeBetweenWhistleAndGCPlaying;
  }
  else if(theGameInfo.state != gameStateLastFrame)
  {
    if(theGameInfo.state == STATE_READY)
      timeWhenLastReadyStarted = theFrameInfo.time;
    else if(theGameInfo.state == STATE_SET)
      timeWhenLastSetStarted = theFrameInfo.time;
    else if(theGameInfo.state == STATE_PLAYING)
      timeWhenLastPlayingStarted = theFrameInfo.time;
  }

  if(theGameInfo.setPlay != setPlayLastFrame)
  {
    if(theGameInfo.setPlay != SET_PLAY_NONE)
      timeWhenLastFreeKickStarted = theFrameInfo.time;
    else
      timeWhenLastFreeKickEnded = theFrameInfo.time;
  }

  if(theBehaviorStatus.searchesForBall())
    extendedGameInfo.timeWhenLastSearchedBall = theFrameInfo.time;

  if(penaltyLastFrame != PENALTY_NONE && !theGameInfo.isPenalized())
    timeWhenLastPenaltyEnded = theFrameInfo.time;

  if(theGameInfo.state != gameStateLastFrame)
    gameStateBeforeCurrent = gameStateLastFrame;
  if(theGameInfo.gamePhase != gamePhaseLastFrame)
    gamePhaseBeforeCurrent = gamePhaseLastFrame;

  // Calibration starts when the mode switches to calibration and we are in playing,
  // or when the calibration mode is active and we switched to playing.
  if ((theGameInfo.playerMode != modeLastFrame && theGameInfo.playerMode == GameInfo::calibration && theGameInfo.state == STATE_PLAYING) || (theGameInfo.playerMode == GameInfo::calibration && theFrameInfo.getTimeSince(timeWhenLastPlayingStarted) == 0))
  {
    extendedGameInfo.startingCalibration = true;
    timeWhenLastCalibrationStarted = theFrameInfo.time;
  }
  else
    extendedGameInfo.startingCalibration = false;

  // DO NOT LET ROBOTS ENTER IN THE READY AFTER A TIMEOUT WHICH DID NOT ALREADY RECEIVE PACKETS DURING THE TIMEOUT!!!
  extendedGameInfo.walkingInFromSidelines =
      (theGameInfo.state == STATE_READY) &&
      ((gameStateBeforeCurrent == STATE_INITIAL) || (gameStateBeforeCurrent == STATE_STANDBY)) &&
      (static_cast<unsigned int>(std::max<int>(theGameInfo.secsRemaining * 1000, 0)) > (fullHalfDuration - 20*1000) ||
       gamePhaseBeforeCurrent == GAME_PHASE_TIMEOUT);

  extendedGameInfo.timeSinceLastPenaltyEnded = theGameInfo.isPenalized() ? 0 : theFrameInfo.getTimeSince(timeWhenLastPenaltyEnded);
  extendedGameInfo.timeWhenLastReadyStarted = timeWhenLastReadyStarted;
  extendedGameInfo.timeSinceReadyStarted = theGameInfo.state != STATE_READY ? 0 : theFrameInfo.getTimeSince(timeWhenLastReadyStarted);
  extendedGameInfo.timeWhenLastSetStarted = timeWhenLastSetStarted;
  extendedGameInfo.timeSinceSetStarted = theGameInfo.state != STATE_SET ? 0 : theFrameInfo.getTimeSince(timeWhenLastSetStarted);
  extendedGameInfo.timeWhenLastPlayingStarted = timeWhenLastPlayingStarted;
  extendedGameInfo.timeSincePlayingStarted = theGameInfo.state != STATE_PLAYING ? 0 : theFrameInfo.getTimeSince(timeWhenLastPlayingStarted);
  extendedGameInfo.timeWhenLastFreeKickStarted = timeWhenLastFreeKickStarted;
  extendedGameInfo.timeSinceFreeKickStarted = theFrameInfo.getTimeSince(timeWhenLastFreeKickStarted);
  extendedGameInfo.timeWhenLastFreeKickEnded = timeWhenLastFreeKickEnded;
  extendedGameInfo.timeSinceLastFreeKickEnded = theFrameInfo.getTimeSince(timeWhenLastFreeKickEnded);
  extendedGameInfo.timeWhenLastCalibrationStarted = timeWhenLastCalibrationStarted;
  extendedGameInfo.timeSinceLastCalibrationStarted = theFrameInfo.getTimeSince(timeWhenLastCalibrationStarted);
  extendedGameInfo.gameStateLastFrame = gameStateLastFrame;
  extendedGameInfo.gamePhaseLastFrame = gamePhaseLastFrame;
  extendedGameInfo.setPlayLastFrame = setPlayLastFrame;
  extendedGameInfo.penaltyLastFrame = penaltyLastFrame;
  extendedGameInfo.gameStateBeforeCurrent = gameStateBeforeCurrent;
  extendedGameInfo.gamePhaseBeforeCurrent = gamePhaseBeforeCurrent;
  extendedGameInfo.modeLastFrame = modeLastFrame;

  if(theGameInfo.state != STATE_SET)
    manuallyPlaced = false;
  else if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT && !theGameInfo.isPenalized() &&
     theFrameInfo.getTimeSince(timeWhenLastPenaltyEnded) > 5000 && theFallDownState.state == FallDownState::pickedUp)
    manuallyPlaced = true;
  extendedGameInfo.manuallyPlaced = manuallyPlaced;

  if(penaltyLastFrame == PENALTY_NONE && theGameInfo.isPenalized())
  {
    // If this is the first GameController packet, it might be that the software
    // just started and the robot is actually penalized for a longer time.
    if(!receivedGameControllerPacket && !theGameInfo.isPenalized(PENALTY_MANUAL))
      timeWhenPenalized = 0;
    else
      timeWhenPenalized = theFrameInfo.time;
  }
  receivedGameControllerPacket = theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) < 45000 - minPenaltyTime;

  extendedGameInfo.returnFromGameControllerPenalty = false;
  extendedGameInfo.returnFromManualPenalty = false;
  if(theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT)
  {
    if(penaltyLastFrame == PENALTY_MANUAL && !theGameInfo.isPenalized())
      extendedGameInfo.returnFromManualPenalty = true;
    else if ((penaltyLastFrame != PENALTY_NONE) && (penaltyLastFrame != PENALTY_SPL_ILLEGAL_MOTION_IN_SET) &&
             !theGameInfo.isPenalized())
    {
      if (SystemCall::getMode() != SystemCall::physicalRobot)
        extendedGameInfo.returnFromGameControllerPenalty = true;
      else if (theFrameInfo.getTimeSince(timeWhenPenalized) >
               ((penaltyLastFrame == PENALTY_SPL_ILLEGAL_POSITION_IN_SET) ? minPenaltyTimeIP : minPenaltyTime))
        extendedGameInfo.returnFromGameControllerPenalty = true;
    }
  }

  rawGameStateLastFrame = theReceivedGameControlData.state;
  gameStateLastFrame = theGameInfo.state;
  gamePhaseLastFrame = theGameInfo.gamePhase;
  setPlayLastFrame = theGameInfo.setPlay;
  penaltyLastFrame = theGameInfo.playerInfo().penalty;
  modeLastFrame = theGameInfo.playerMode;
}
