/**
 * @file GameInfoProvider.cpp
 * 
 * Modifies the GameInfo with state changes from the Game Controller, 
 * whistle, manual buttons, etc.
 * 
 * Based in part on WhistleHandler and KickoffStateProvider from the 
 * B-Human Code Release 2021.
 * 
 * State possibilities:
 * 
 * GAME_PHASE_TIMEOUT:
 *   Only STATE_INITIAL and SET_PLAY_NONE possible
 * 
 * GAME_PHASE_PENALTY_SHOOT:
 *   Must be SET_PLAY_NONE
 *   STATE_INITIAL - while robots being setup/selected
 *   STATE_SET - robot selected as striker or goalie, at waiting point
 *   STATE_PLAYING - play the penalty shoot kick as striker or goalie
 *   STATE_FINISHED - penalty shot complete
 * 
 * GAME_PHASE_NORMAL:
 *   STATE_INITIAL - must be SET_PLAY_NONE
 *   STATE_READY
 *     SET_PLAY_NONE - walk to attack/defence kickoff positions
 *     SET_PLAY_PENALTY_KICK - get into position for a penalty kick (30 seconds)
 * 
 *   STATE_SET
 *     SET_PLAY_NONE - wait at kickoff position (for whistle), can look around but otherwise not move
 *     SET_PLAY_PENALTY_KICK - wait at penalty kick waiting point (for whistle), can look around but otherwise not move
 *     Other set plays might rarely occur if their cause occurs during the 15 secs before GC signals playing after 
 *       whistle. Behaviour the same as under STATE_PLAYING in these cases.
 * 
 *   STATE_PLAYING
 *     SET_PLAY_NONE - 15 secs after whistle to signal kickoff, or when a set play ends, play normally
 *     SET_PLAY_PENALTY_KICK - 15 seconds after whistle to signal penalty, start if not already started, 15 more secs to complete
 *     SET_PLAY_PUSHING_FREE_KICK - 30 seconds (or first touch) until ball is free and set play ends
 *     SET_PLAY_KICK_IN - 30 seconds (or first touch) until ball is free and set play ends
 *     SET_PLAY_GOAL_KICK - 30 seconds (or first touch) until ball is free and set play ends
 *     SET_PLAY_CORNER_KICK - 30 seconds (or first touch) until ball is free and set play ends
 * 
 * Whistle signals:
 *   guessed STATE_PLAYING after STATE_SET 
 *     kickoff - 10 secs after whistle (or first touch) is ball free, 15 secons from whistle until 
 *               GC STATE_PLAYING unambiguously ends the kickoff
 *     penalty kick/penalty shootout kick - 15 secs after whistle, GC STATE_PLAYING is signalled and there are
 *                                          15 more secs to complete the kick
 *   guessed STATE_READY after STATE_PLAYING
 *     goal scored - 15 secs after the whistle, GC STATE_PLAYING switches to STATE_READY and the score changes
 * 
 * Visual signals:
 *   guessed STATE_READY after STANDBY
 *     visually signalled ready - 30 seconds after the signal GC STATE_READY will be signalled and there will be
 *                                max 15 secs remaining to get positioned.
 * 
 * @author Rudi Villing
 */

#include "GameInfoProvider.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/TextLogging.h"


MAKE_MODULE(GameInfoProvider, infrastructure);

DECL_TLOGGER(tlogger, "GameInfoProvider", TextLogging::INFO);


void GameInfoProvider::update(GameInfo& theGameInfo)
{
  if (!initialised)
  {
    resetGameInfo(theGameInfo);
    initialised = true;
  }

  // TLOGV(tlogger, "update called");

  // we don't allow unstiff in sim
  if ((theGameInfo.playerMode == GameInfo::unstiff) && (SystemCall::getMode() != SystemCall::physicalRobot))
    theGameInfo.playerMode = GameInfo::active;

  handlePlayerModeButtons(theGameInfo);

  // update basic fields from rawGameControl
  updateGameInfoFromReceivedGameControlData(theGameInfo);

  // no need to do anything with GC data if the player is unstiff or calibrating
  // so if the player is not active, there is nothing more to do here
  if (theGameInfo.playerMode != GameInfo::active)
    return;

  // is our guessed state a mistake?
  updateIllegalMotionPenalties(theGameInfo);
  updateEndOfGuessedState(theGameInfo);

  // check whistle overrides depending on state, confirm with ball in goal if needed
  updateGuessedState(theGameInfo);

  // check kickoff and set play ball free
  updateBallMotion(theGameInfo);
  updateKickoffAndSetPlayBallFree(theGameInfo);

  // if game controller inactive, check for manual states
  if (!theGameInfo.gcActive)
    handleManualStateButtons(theGameInfo);
}


void GameInfoProvider::updateGameInfoFromReceivedGameControlData(GameInfo& gameInfo)
{
  // we update each frame, whether we have a new packet or not - it will only trigger on change
  receivedGameStateTrigger.update(theReceivedGameControlData.state);
  receivedSetPlayTrigger.update(theReceivedGameControlData.setPlay);


  if (theFrameInfo.getTimeSince(theReceivedGameControlData.timeLastPacketReceived) < gameControllerTimeout)
  {
    gameInfo.gcActive = true;

    if (timeLastPacketReceived == theReceivedGameControlData.timeLastPacketReceived) // not a new packet?
      return;
    else
      timeLastPacketReceived = theReceivedGameControlData.timeLastPacketReceived;
  }
  else
  {
    gameInfo.gcActive = false;
    return; // no new info so no updating required
  }

  // save values we might have guessed/overridden
  uint8_t state = gameInfo.state;

  static_cast<ReceivedGameControlData&>(gameInfo) = theReceivedGameControlData;

  if (gameInfo.stateIsGuessed)
  {
    // reapply our guesses and save the current GC values
    gameInfo.state = state;
  }

  if (receivedGameStateTrigger.valueChanged())
  {
    timeWhenReceivedStateChanged = theFrameInfo.time;

    if (!gameInfo.stateIsGuessed)
      timeWhenStateChanged = theFrameInfo.time;
  }

  gameInfo.playingLeftToRight = theReceivedGameControlData.isPlayingLeftToRight();
}


void GameInfoProvider::updateIllegalMotionPenalties(GameInfo& gameInfo)
{
  illegalMotionTimes.resize(MAX_NUM_PLAYERS * 2, 0); // space for 2 teams

  // when looking at illegal motion penalties after we've guessed a state change
  // it doesn't matter which team gets penalized - it is still giving us information
  // that the guess was incorrect

  anyIllegalMotion = false;

  // illegal motion only matters if we are currently guessing the state
  if (gameInfo.stateIsGuessed &&
      ((theReceivedGameControlData.state == STATE_SET) || (theReceivedGameControlData.state == STATE_STANDBY)))
  {
    unsigned penalty = (theReceivedGameControlData.state == STATE_SET) ? PENALTY_SPL_ILLEGAL_MOTION_IN_SET
                                                                       : PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY;

    for (int iTeam = 0; iTeam < 2; iTeam++)
    {
      for (unsigned iPlayer = 0; iPlayer < gameInfo.playersPerTeam; iPlayer++)
      {
        const int i = iTeam * MAX_NUM_PLAYERS + iPlayer;

        if (gameInfo.teams[iTeam].players[iPlayer].penalty != penalty)
          illegalMotionTimes[i] = 0;
        else if (illegalMotionTimes[i] == 0)
          illegalMotionTimes[i] = theFrameInfo.time;
      }
    }

    for (unsigned penaltyTime : illegalMotionTimes)
      if (penaltyTime > timeWhenStateChanged) // for guessed state
        anyIllegalMotion = true;
  }
}


void GameInfoProvider::resetBallMotion()
{
  ballMoved = false;
//   ballOutOfCenterCircle = false;
// 
//   ballOutOfCenterCircleCounter = 0;
  ballPositions.clear();
  lastBallPositionTime = 0;
}

void GameInfoProvider::updateBallMotion(GameInfo& gameInfo)
{
//   // only buffer the ball in the playing state and only when it should be
//   // stationary, e.g. during kickoff, penalty kick, or the start of a set play
// 
//   if (gameInfo.state != STATE_PLAYING)
//   {
//     ballMoved = false;
//     ballOutOfCenterCircle = false;
// 
//     ballOutOfCenterCircleCounter = 0;
//     ballPositions.clear();
//     lastBallPositionTime = 0;
//     return;
//   }

  // do we have a valid ball? We use the most recently seen ball (if new enough)
  // from the ball model or the teamBallModel otherwise.
  // WARNING: hopefully the team ball model will be stable enough with coarse
  // ball positions being communicated and communication happening rarely 
  Vector2f ballPositionOnField;
  if (theBallModel.timeWhenLastSeen > lastBallPositionTime)
    ballPositionOnField = theRobotPose.toFieldCoordinates(theBallModel.estimate.position);
  else if (theTeamBallModel.isValid)
    ballPositionOnField = theRobotPose.toFieldCoordinates(theTeamBallModel.position);
  else
    return; // no useful ball model so nothing to do here

  // ---------------------
  // ball in center circle 
  // ---------------------

  // any set play means the ball is no longer in the center circle
  // if (gameInfo.setPlay != SET_PLAY_NONE)
  //   ballOutOfCenterCircle = true;

  // handle ball leaving center circle at kickoff
//   if (!ballOutOfCenterCircle)
//   {
//     if (ballPositionOnField.norm() > theFieldDimensions.centerCircleRadius + theFieldDimensions.fieldLinesWidth * 0.5f +
//                                          theBallSpecification.radius + ballOutOfCenterCircleTolerance)
//       ballOutOfCenterCircleCounter++;
//     else if (ballOutOfCenterCircleCounter > 0)
//       ballOutOfCenterCircleCounter--;
// 
//     if (ballOutOfCenterCircleCounter > ballOutOfCenterCircleCounterThreshold)
//       ballOutOfCenterCircle = true; // will only be set back to false at the start of the next kickoff
//   }

  // -------------
  // ball movement 
  // -------------

  if (!ballMoved) // ballMoved will be set to false at the start of any play where we want to track ball movement
  {
    const bool isPenaltyKick =
      ((gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) || (gameInfo.setPlay == SET_PLAY_PENALTY_KICK));
    const float closeToRobotDistance = theFieldDimensions.centerCircleRadius - ballMovedCloseToRobotThreshold;

    // Did this robot kick the ball?
    if (theMotionInfo.lastKickTimestamp > timeWhenStateChanged)
    {
      ballMoved = true;
    }
    else if (!gameInfo.isOurKick() && !isPenaltyKick &&
             //  ((theBallModel.timeWhenLastSeen > timeWhenStateChanged) ||
             ((kickoffStartTime && (Time::getTimeBetween(theBallModel.timeWhenLastSeen, kickoffStartTime) > 0)) ||
              (setPlayStartTime && (Time::getTimeBetween(theBallModel.timeWhenLastSeen, setPlayStartTime) > 0))) &&
             (theBallModel.estimate.position.squaredNorm() < sqr(closeToRobotDistance)))
    {
      ballMoved = true;
    }
    // If robot is moving, the buffer content is invalid:
    else if ((theMotionInfo.executedPhase != MotionPhase::stand) ||
             (theGyroState.deviation.y() > standStillGyroThreshold))
    {
      ballPositions.clear();
    }
    else if (Time::getTimeBetween(lastBallPositionTime + ballSaveInterval, theBallModel.timeWhenLastSeen) > 0) // only add at intervals
    {
      ballPositions.push_front(theBallModel.lastPerception);
      lastBallPositionTime = theFrameInfo.time;

      if (ballPositions.size() >= 6)
      {
        const auto medianOfThree = [](const Vector2f &a, const Vector2f &b, const Vector2f &c) -> const Vector2f &
        {
          const float aN = a.squaredNorm();
          const float bN = b.squaredNorm();
          const float cN = c.squaredNorm();

          if (aN >= bN)
          {
            if (cN >= aN)
              return a;
            else // c < a
              return bN >= cN ? b : c;
          }
          else // b > a
          {
            if (cN >= bN)
              return b;
            else // c < b
              return aN >= cN ? a : c;
          }
        };

        const size_t firstIndex = ballPositions.size() - 1;

        const Vector2f &start =
            medianOfThree(ballPositions.back(), ballPositions[firstIndex - 1], ballPositions[firstIndex - 2]);

        const Vector2f &end = medianOfThree(ballPositions.front(), ballPositions[1], ballPositions[2]);

        if ((start - end).squaredNorm() > sqr(ballMovedTolerance))
          ballMoved = true;
      }
    }
  }
}


void GameInfoProvider::startKickoff(GameInfo& gameInfo)
{
  gameInfo.ballFreeState = GameInfo::notFreeKickoff;
  kickoffStartTime = theFrameInfo.time;
  setPlayStartTime = 0;

  resetBallMotion();
}

void GameInfoProvider::startSetPlay(GameInfo& gameInfo)
{
  gameInfo.ballFreeState = GameInfo::notFreeSetPlay;
  kickoffStartTime = 0;
  setPlayStartTime = theFrameInfo.time;

  resetBallMotion();
}

void GameInfoProvider::setBallFree(GameInfo& gameInfo)
{
  gameInfo.ballFreeState = GameInfo::ballIsFree;
  kickoffStartTime = 0;
  setPlayStartTime = 0;
  
  ballMoved = true;
}

void GameInfoProvider::updateKickoffAndSetPlayBallFree(GameInfo& gameInfo)
{
  const bool isPenaltyKick =
    ((gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) || (gameInfo.setPlay == SET_PLAY_PENALTY_KICK));

  // have we just started a kickoff period (just transitioned to playing AND not doing a penalty)?
  // (Note: we include a number of different transitions to allow for some direct transitions
  // in SimRobot that wouldn't happen in the GC)
  if (!gameInfo.isKickoff() && !isPenaltyKick && (gameInfo.state == STATE_PLAYING) &&
      ((receivedGameStateTrigger.prevValue == STATE_SET) || (receivedGameStateTrigger.prevValue == STATE_READY) ||
       (receivedGameStateTrigger.prevValue == STATE_STANDBY) || (receivedGameStateTrigger.prevValue == STATE_INITIAL)) &&
      (gameInfo.setPlay == SET_PLAY_NONE))
  {
    startKickoff(gameInfo);
  }
  // have we just started a set play?
  else if (receivedSetPlayTrigger.valueChanged() && (receivedSetPlayTrigger.value != SET_PLAY_NONE))
  {
    startSetPlay(gameInfo);
  }
  // are we doing a kickoff already and if so, is it finished ?
  else if (gameInfo.isKickoff())
  {
    // a kickoff is finished when the GC transitions to playing after a whistle
    // OR when the max kickoff time has elapsed
    // OR when the ball has moved (enough)
    const bool gcSwitchedToPlaying =
        (receivedGameStateTrigger.valueChanged() && (receivedGameStateTrigger.value == STATE_PLAYING));
    const bool kickoffTimeElapsed = (theFrameInfo.getTimeSince(kickoffStartTime) > theGameConfig.kickoffDuration);

    if (gcSwitchedToPlaying || kickoffTimeElapsed || ballMoved)
      setBallFree(gameInfo);
  }
  // are we doing a set play and if so, is it finished?
  else if (!gameInfo.isBallFree())
  {
    if ((gameInfo.setPlay == SET_PLAY_NONE) || ballMoved)
      setBallFree(gameInfo);
  }
}

void GameInfoProvider::updateEndOfGuessedState(GameInfo& gameInfo)
{
  if (!gameInfo.gcActive || !gameInfo.stateIsGuessed)
    return;

  // everything from here on is working with a guessed game state

  // has the GC game state changed?
  if (receivedGameStateTrigger.valueChanged())
  {
    gameInfo.stateIsGuessed = false;
    gameInfo.state = theReceivedGameControlData.state;
    timeWhenStateChanged = theFrameInfo.time;
    setBallFree(gameInfo);

    ANNOTATION_FMT("GameInfoProvider", "GC state changed to {} => back to GC state", gameInfo.state);
    return;
  }
  // Stop overriding when both game states match. -- TODO can this condition ever be true without the above being true first?
  else if (theReceivedGameControlData.state == gameInfo.state)
  {
    gameInfo.stateIsGuessed = false;
    // leave timeWhenStateChanged, ballFreeInfo alone

    ANNOTATION_FMT("GameInfoProvider", "GC state {} now matches our guess", theReceivedGameControlData.state);
  }

  // correct any whistle related mistakes
  if (gameInfo.stateIsGuessed && (useWhistleForKickOff || useWhistleAfterGoal))
  {
    // Check GameController messages for wrong switch to READY after a guessed goal whistle
    // - case 1: GC does not send ready signal after 15 secs + operator delay
    // - case 2: GC indicates any set play in playing state (for longer than the operator delay) so we couldn't be in ready state
    if ((gameInfo.state == STATE_READY) && (theReceivedGameControlData.state == STATE_PLAYING) &&
        ((theFrameInfo.getTimeSince(timeWhenStateChanged) > (theGameConfig.goalToReadyDelay + gameControllerOperatorDelay)) ||
         ((theReceivedGameControlData.setPlay != SET_PLAY_NONE) &&
          (theFrameInfo.getTimeSince(timeWhenStateChanged) > gameControllerOperatorDelay))))
    {
      gameInfo.stateIsGuessed = false;
      gameInfo.state = theReceivedGameControlData.state;
      timeWhenStateChanged = timeWhenReceivedStateChanged;
      setBallFree(gameInfo);

      if (sayGuessedStateChange)
        SystemCall::say("Back to Game Controller playing state");

      ANNOTATION("GameInfoProvider", "READY state guess was incorrect => back to GC PLAYING state {}");
    }
    // check for wrong switch to PLAYING after guessed whistle in SET
    else if ((gameInfo.state == STATE_PLAYING) && (theReceivedGameControlData.state == STATE_SET) && anyIllegalMotion)
    {
      gameInfo.stateIsGuessed = false;
      gameInfo.state = theReceivedGameControlData.state;
      // gameInfo.ballFreeState = GameInfo::ballIsFree;
      timeWhenStateChanged = timeWhenReceivedStateChanged;
      setBallFree(gameInfo);

      if (sayGuessedStateChange)
        SystemCall::say("Back to Game Controller set state");

      ANNOTATION("GameInfoProvider", "illegal motion penalty => back to GC SET state");
    }
  }

  // correct any visual signal mistakes
  if (gameInfo.stateIsGuessed && useVisualReady)
  {
    if ((gameInfo.state == STATE_READY) && (theReceivedGameControlData.state == STATE_STANDBY) && anyIllegalMotion)
    {
      gameInfo.stateIsGuessed = false;
      gameInfo.state = theReceivedGameControlData.state;
      timeWhenStateChanged = timeWhenReceivedStateChanged;

      if (sayGuessedStateChange)
        SystemCall::say("Back to Game Controller standby state");

      ANNOTATION("GameInfoProvider", "illegal motion penalty => back to GC STANDBY state");
    }
  }
}

bool GameInfoProvider::checkVisualReadySignalled(GameInfo& gameInfo)
{
  if (gameInfo.state != STATE_STANDBY)
    return false;

  // is detected by self?
  if (theStandbyReadyGesture.isDetected ||
      (theFrameInfo.getTimeSince(theStandbyReadyGesture.frameTimeLastDetection) <= visualReadyMatchDuration))
    return true;
  // detected by any teammate
  else
  {
    for (Teammate* pTeammate : theTeamData.activeTeammates)
    {
      int timeSinceTeammateGestureDetected =
          theFrameInfo.getTimeSince(pTeammate->theStandbyReadyGesture.frameTimeLastDetection);

      if (timeSinceTeammateGestureDetected < visualReadyMatchDuration)
        return true;
    }

    // no teammate has a recent signal
    return false;
  }
}



bool GameInfoProvider::checkWhistle(GameInfo& gameInfo)
{
  (void)gameInfo; // force use for now

  // if (gameInfo.kickoff && (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) < ignoreWhistleAfterKickOffMs))
  //   return false;
  // else if (((gameInfo.setPlay == SET_PLAY_PENALTY_KICK) || (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)) &&
  //          (gameInfo.state == STATE_PLAYING) &&
  //          (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) < ignoreWhistleAfterPenaltyKickMs))
  //   return false;

  if (theFrameInfo.getTimeSince(lastWhistleTime) < ignoreWhistleDuration)
    return false;

  int consensus = 0;

  if (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) <= whistleMatchDuration)
    ++consensus;
  else if (requireOwnWhistle)
    return false;

  if (consensus < requiredWhistleConsensus)
  {
    for (auto teammate : theTeamData.teammates)
    {
      if (theFrameInfo.getTimeSince(teammate.theWhistle.lastTimeWhistleDetected) <= whistleMatchDuration)
        ++consensus;
    }
  }

  if (consensus >= requiredWhistleConsensus)
  {
    lastWhistleTime = theFrameInfo.time;
    return true;
  }
  else
    return false;
}


void GameInfoProvider::updateGuessedState(GameInfo& gameInfo)
{
  bool whistleDetected = checkWhistle(gameInfo);

  const bool isPenaltyKick =
      ((gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) || (gameInfo.setPlay == SET_PLAY_PENALTY_KICK));

  // Switching from GC SET --> guessed PLAYING (and possibly kickoff):
  //   when the GameController is sending SET, and we are not guessing yet, and we heard a whistle.
  if (useWhistleForKickOff && whistleDetected && (gameInfo.state == STATE_SET) && !gameInfo.stateIsGuessed)
  {
    gameInfo.stateIsGuessed = true;
    gameInfo.state = STATE_PLAYING;
    timeWhenStateChanged = theFrameInfo.time;

    // timeOfLastStateChange =
    //     theFrameInfo.time +
    //     (theReceivedGameControlData.setPlay == SET_PLAY_PENALTY_KICK ? ignoreWhistleAfterPenaltyKick : ignoreWhistleAfterKickOff);

    if (!isPenaltyKick)
    {
      startKickoff(gameInfo);
      TLOGI(tlogger, "Whistle detected => kickoff");

      if (sayGuessedStateChange)
        SystemCall::say("whistle Kickoff");
    }
    else 
    {
      TLOGI(tlogger, "Whistle detected => playing");
      if (sayGuessedStateChange)
        SystemCall::say("whistle playing");
    }
  }
  // Switching from PLAYING (actual/guessed) to READY because a goal was scored:
  //   Did we hear a whistle in playing state and was the ball in the goal (if we are using that check)?
  else if (useWhistleAfterGoal && whistleDetected && (gameInfo.state == STATE_PLAYING) && !isPenaltyKick && checkBallInGoal())
  {
    gameInfo.stateIsGuessed = true;
    gameInfo.state = STATE_READY;
    timeWhenStateChanged = theFrameInfo.time;

    // Even though theBallInGoal might be some seconds old, we accept it without further check because
    // the ball could be hidden for a while before the referee finally whistles.

    gameInfo.kickingTeam = theBallInGoal.inOwnGoal ? gameInfo.ourTeam().teamNumber : gameInfo.opponentTeam().teamNumber;

    if (sayGuessedStateChange)
      SystemCall::say(fmt::format(
          "Whistle Goal for {}",
          TypeRegistry::getEnumName(static_cast<Settings::TeamColor>(
              gameInfo.isOurKick() ? gameInfo.ourTeam().fieldPlayerColor : gameInfo.opponentTeam().fieldPlayerColor))));
  }
  else if ((gameInfo.state == STATE_STANDBY) && checkVisualReadySignalled(gameInfo))
  {
    gameInfo.stateIsGuessed = true;
    gameInfo.state = STATE_READY;
    timeWhenStateChanged = theFrameInfo.time;

    TLOGI(tlogger, "Visual ready signal detected => READY");

    if (sayGuessedStateChange)
      SystemCall::say("visual ready");
  }
}

bool GameInfoProvider::checkBallInGoal()
{
  //was in goal in the last 5 seconds
  return !useBallInGoalCheck || theBallInGoal.timeSinceLastInGoal <= acceptBallInGoalDelay;
}


void GameInfoProvider::handlePlayerModeButtons(GameInfo& gameInfo)
{
  // when true, ignoreChestButton will suppress the chest button in handleManualStateButtons
  ignoreChestButton = false;

  if (!(theReceivedGameControlData.gamePhase != GAME_PHASE_PENALTYSHOOT && theReceivedGameControlData.state == STATE_FINISHED))
    whenNotInFinishedState = theFrameInfo.time;

  switch (gameInfo.playerMode)
  {
    default:
    case GameInfo::unstiff: // UNSTIFF to ACTIVE
      if(theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      { 
        resetGameInfo(gameInfo);
        TLOGI(tlogger, "Chestbutton pressed => mode UNSTIFF --> ACTIVE, GC state {}", gameInfo.getStateString());       
        gameInfo.playerMode = GameInfo::active;
        ignoreChestButton = true; // suppress the chest button in this cognition cycle -- see handleManualStateButtons
      }
      break;

    case GameInfo::active: // ACTIVE --> UNSTIFF or CALIBRATION
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theFrameInfo.getTimeSince(whenNotInFinishedState) > finishedToUnstiffDuration)
      {
        if (theFrameInfo.getTimeSince(whenNotInFinishedState) > finishedToUnstiffDuration)
          TLOGI(tlogger, "in FINISHED state for long enough => mode ACTIVE --> UNSTIFF");
        else      
          TLOGI(tlogger, "head buttons pressed => mode ACTIVE --> UNSTIFF");

        resetGameInfo(gameInfo);
        gameInfo.playerMode = GameInfo::unstiff;
      }
      else if(theReceivedGameControlData.state == STATE_INITIAL &&
              !theReceivedGameControlData.playerInfo().isPenalized() &&
              theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
              theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
      {
        TLOGI(tlogger, "front head and chest buttons pressed => mode ACTIVE --> CALIBRATION");

        resetGameInfo(gameInfo);
        gameInfo.playerMode = GameInfo::calibration;
        gameInfo.state = STATE_PLAYING;
      }
      break;

    case GameInfo::calibration: // CALIBRATION --> UNSTIFF
      if((theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration &&
          theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration) ||
         theBehaviorStatus.activity == BehaviorStatus::calibrationFinished)
      {
        TLOGI(tlogger, "head buttons pressed => mode CALIBRATION --> UNSTIFF");
        
        resetGameInfo(gameInfo);
        gameInfo.playerMode = GameInfo::unstiff;
      }
      break;
  }
}


void GameInfoProvider::handleManualStateButtons(GameInfo& gameInfo)
{
  // this function only runs when the playerMode is active
  ASSERT(gameInfo.playerMode == GameInfo::active);
  
  ReceivedGameControlData::TeamInfo& ourTeam = gameInfo.ourTeam();
  ReceivedGameControlData::RobotInfo& playerInfo = ourTeam.getPlayer(gameInfo.playerIndex());

  // update config (but only in INITIAL)

  if (gameInfo.state == STATE_INITIAL && !playerInfo.isPenalized())
  {
    if (theEnhancedKeyStates.hitStreak[KeyStates::lFootLeft] == 1)
    {
      ourTeam.fieldPlayerColor = (ourTeam.fieldPlayerColor + 1) % (TEAM_GRAY + 1); // cycle between TEAM_BLUE .. TEAM_GRAY

      switch (ourTeam.fieldPlayerColor)
      {
        case TEAM_BLUE: SystemCall::say("Team blue"); break;
        case TEAM_RED: SystemCall::say("Team red"); break;
        case TEAM_YELLOW: SystemCall::say("Team yellow"); break;
        case TEAM_BLACK: SystemCall::say("Team black"); break;
        case TEAM_WHITE: SystemCall::say("Team white"); break;
        case TEAM_ORANGE: SystemCall::say("Team orange"); break;
        case TEAM_PURPLE: SystemCall::say("Team purple"); break;
        case TEAM_BROWN: SystemCall::say("Team brown"); break;
        case TEAM_GRAY: SystemCall::say("Team gray"); break;
        default:
        case TEAM_GREEN: SystemCall::say("Team green"); break;
      }
    }

    if (theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
    {
      if (gameInfo.gamePhase == GAME_PHASE_NORMAL)
      {
        gameInfo.gamePhase   = GAME_PHASE_PENALTYSHOOT;
        gameInfo.kickingTeam = ourTeam.teamNumber;
        SystemCall::say("Penalty striker");
      }
      else if ((gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) && (gameInfo.kickingTeam == ourTeam.teamNumber))
      {
        gameInfo.kickingTeam = 0; // the opponent team but we don't know their team number
        SystemCall::say("Penalty keeper");
      }
      else
        gameInfo.gamePhase = GAME_PHASE_NORMAL;
    }
  }

  // update manual player state

  if (!ignoreChestButton && theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
  {
    if (playerInfo.penalty == PENALTY_NONE)
      playerInfo.penalty = PENALTY_MANUAL; // Note that the first manual penalty doesn't yet switch the robot 
                                       // out of INITIAL state (if there is no GC)
    else
    {
      playerInfo.penalty = PENALTY_NONE;
      gameInfo.state = STATE_PLAYING;
    }
  }
}


void GameInfoProvider::resetGameInfo(GameInfo& gameInfo)
{
  std::memset(&static_cast<RoboCup::RoboCupGameControlData&>(gameInfo), 0, sizeof(gameInfo));

  gameInfo.teams[0].teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  gameInfo.teams[0].fieldPlayerColor = static_cast<uint8_t>(Global::getSettings().fieldPlayerColor);
  gameInfo.teams[0].goalkeeperColor = static_cast<uint8_t>(Global::getSettings().goalkeeperColor);
  gameInfo.teams[0].goalkeeper = 1; // default to player 1 for now (TODO: maybe in the settings later)

  gameInfo.teams[1].fieldPlayerColor = gameInfo.teams[0].fieldPlayerColor ^ 1; // we don't know better
  gameInfo.teams[1].goalkeeperColor = gameInfo.teams[0].goalkeeperColor ^ 1; // we don't know better
  
  gameInfo.firstHalf = 1;

  gameInfo.gcActive = theFrameInfo.getTimeSince(gameInfo.timeLastPacketReceived) < gameControllerTimeout;

  gameInfo.playerNumber = Global::getSettings().playerNumber;
  gameInfo.playerMode = GameInfo::unstiff;
  gameInfo.playingLeftToRight = true;


  timeWhenStateChanged = 0;
  timeWhenReceivedStateChanged = 0;

  whenNotInFinishedState = theFrameInfo.time;

  receivedGameStateTrigger.reset(STATE_INITIAL);
  receivedSetPlayTrigger.reset(SET_PLAY_NONE);

  kickoffStartTime = 0;
  setPlayStartTime = 0;


  TLOGI(tlogger, "resetGameInfo: player index {}, number {}", gameInfo.playerIndex(), gameInfo.playerNumber);
}
