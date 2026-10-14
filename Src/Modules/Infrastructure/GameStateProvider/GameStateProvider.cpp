/**
 * @file GameStateProvider.cpp
 *
 * This file implements a module that provides a condensed representation of the game state.
 *
 * @author Arne Hasselbring
 */

#include "GameStateProvider.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Debugging/Annotation.h"
#include "TextLogging/TextLogging.h"

#include <algorithm>


static_assert(GAMECONTROLLER_STRUCT_VERSION == 20,
              "This code was designed for the 2026 Game Controller packet (v20). "
              "Review all field/enum mappings before updating this assert.");

MAKE_MODULE(GameStateProvider);

DECL_TLOGGER(tlogger, "GameStateProvider", TextLogging::DEBUG);

static constexpr int NUM_TEAMS = 2;

// convenience shorthand for outputting to text log and annotation simultaneously
#define TLOGI_ANNOTATION(logger_, ...)                                                                                 \
  do                                                                                                                   \
  {                                                                                                                    \
    TLOGI(logger_, "*** " __VA_ARGS__);                                                                                \
    ANNOTATION_FMT("GameStateProvider", __VA_ARGS__);                                                                  \
  } while (0)


void GameStateProvider::update(GameState& gameState)
{
  gameState.getStateDuration = [this, &gameState]() -> int
  {
    return theFrameInfo.getTimeSince(gameState.timeWhenStateStarted) - gameState.stoppedTimeInState;
  };

  gameState.getPlayerStateDuration = [this, &gameState]() -> int
  {
    return theFrameInfo.getTimeSince(gameState.timeWhenPlayerStateStarted) - gameState.stoppedTimeInPlayerState;
  };


  DECLARE_DEBUG_DRAWING("module:GameStateProvider:ballCovariance", "drawingOnField");


  auto setGameState = [this, &gameState](GameState::State s)
  {
    gameState.state = s;
    gameState.timeWhenStateStarted = theFrameInfo.time; // override after lambda if needed
    gameState.timeWhenStateEnds = 0; // override after lambda if needed
    gameState.stoppedTimeInState = 0;
    prevStoppedTimeInCurrentState = 0;
  };

  auto setPlayerState = [this, &gameState](GameState::PlayerState s, bool forceStart = true)
  {
    if (forceStart || (gameState.playerState != s))
    {
      gameState.playerState = s;
      gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
      gameState.stoppedTimeInPlayerState = 0;
      prevStoppedTimeInCurrentPlayerState = 0;
    }
  };

  auto setToUnstiff = [this, &gameState](const char* reason)
  {
    reset(gameState);
    TLOGI_ANNOTATION(tlogger, "playerState ==> unstiff (because {}) state={}",
                     reason, TypeRegistry::getEnumName(gameState.state));
  };

  auto setToPrepare = [this, &gameState, &setPlayerState]
  {
    reset(gameState);
    setPlayerState(GameState::prepare);
    TLOGI_ANNOTATION(tlogger, "playerState ==> prepare, state={}", TypeRegistry::getEnumName(gameState.state));
  };

  auto setToCalibration = [this, &gameState, &setGameState, &setPlayerState]
  {
    reset(gameState);
    setGameState(GameState::playing);
    setPlayerState(GameState::calibration);
    TLOGI_ANNOTATION(tlogger, "playerState ==> calibration, state=playing");
  };

  auto setToActive = [this, &gameState, &setPlayerState]
  {
    reset(gameState);
    setPlayerState(GameState::active);
    timeOfLastIntegratedGameControllerData = theFrameInfo.time - gameControllerTimeout;
    TLOGI_ANNOTATION(tlogger, "playerState ==> active, state={}", TypeRegistry::getEnumName(gameState.state));
  };

  // is there currently a stop active?
  if (timeWhenStopActivated)
  {
    gameState.stoppedTimeInState =
        prevStoppedTimeInCurrentState +
        theFrameInfo.getTimeSince(std::max(timeWhenStopActivated, gameState.timeWhenStateStarted));
    gameState.stoppedTimeInPlayerState =
        prevStoppedTimeInCurrentPlayerState +
        theFrameInfo.getTimeSince(std::max(timeWhenStopActivated, gameState.timeWhenPlayerStateStarted));
  }
  gameState.stopped = theGameControllerData.stopped;
  if (gameState.stopped)
  {
    if (timeWhenStopActivated == 0) // did stop just became active now?
      timeWhenStopActivated = theFrameInfo.time; 
  } 
  else // not stopped
  {
    if (timeWhenStopActivated) // was stop active and is just now deactivated
    {
      prevStoppedTimeInCurrentState += theFrameInfo.getTimeSince(timeWhenStopActivated);
      prevStoppedTimeInCurrentPlayerState += theFrameInfo.getTimeSince(timeWhenStopActivated);
      timeWhenStopActivated = 0; // stop no longer active
    }
  }


  DEBUG_RESPONSE_ONCE("module:GameStateProvider:unstiff")
    setToUnstiff("DEBUG_REQUEST");

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:prepare")
    setToPrepare();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:calibration")
    setToCalibration();

  DEBUG_RESPONSE_ONCE("module:GameStateProvider:active")
  {
    setToActive();
    timeOfLastIntegratedGameControllerData = theFrameInfo.time - 1;
  }

  if(gameState.state != GameState::afterHalf)
    timeWhenStateNotAfterHalf = theFrameInfo.time;

  gameState.whistled = false;

  TLOGV(tlogger, "start of update: GameState {}, PlayerState {}", TypeRegistry::getEnumName(gameState.state),
        TypeRegistry::getEnumName(gameState.playerState));

  if (robotSafetyStateEdge.update(theRobotSafetyState.state))//, RobotSafetyState::operational))
  {
    if (theRobotSafetyState.state == RobotSafetyState::operational)
      timeWhenRobotBecameOperational = theFrameInfo.time;

    TLOGD(tlogger, "robotSafetyState just became {}, playerState is {}",
          TypeRegistry::getEnumName(theRobotSafetyState.state), TypeRegistry::getEnumName(gameState.playerState));
  }

  if (Global::getSettings().robotType == Settings::nao)
    updateFromNaoHri(gameState);
  else if (Global::getSettings().robotType == Settings::k1)
    updateFromBoosterK1Hri(gameState);

  if (theRobotSafetyState.state == RobotSafetyState::operational)
  {
    switch (gameState.playerState)
    {
    case GameState::unstiff:
      if (activeModeRequested)
        setToActive();
      else if (prepareModeRequested)
        setToPrepare();
      break;

    case GameState::prepare:
      // F1 (e-stop) must always override, from any state.
      if (unstiffModeRequested)
        setToUnstiff("HRI requested");
      // Leave prepare either via the manual WALK button (activeModeRequested) or automatically
      // once a real GameController packet for our team has actually arrived - a robot that reaches
      // prepare (stood up via the STAND button) should not need a human to also press WALK before
      // it will follow a game that's already under way. Once active, the normal GC state tracking
      // below picks up whatever the real current state is (initial/ready/set/playing), rather than
      // assuming the game is still at the very start.
      else if (activeModeRequested || theGameControllerData.timeLastPacketReceived != 0)
        setToActive();
      break;

    case GameState::calibration:
      if (unstiffModeRequested)
        setToUnstiff("HRI requested");
      else if ((Global::getSettings().robotType == Settings::nao) && theBehaviorStatus.calibrationFinished)
        setToUnstiff("calibration finished"); // FIXME - not for K1
      break;

    default:
      if (unstiffModeRequested)
        setToUnstiff("HRI requested");
      else if ((SystemCall::getMode() != SystemCall::simulatedRobot) &&
               ((Global::getSettings().robotType == Settings::nao) &&
                (theFrameInfo.getTimeSince(timeWhenStateNotAfterHalf) > unstiffAfterHalfDuration)))
      {
        if (Global::getSettings().robotType == Settings::k1)
          setToPrepare();
        else
          setToUnstiff("after half"); // FIXME - needs to be different for K1
      }
      else if (calibrationModeRequested)
        setToCalibration();
      else if (prepareModeRequested)
        setToPrepare();
    }
  }
  else // safety state not operational
  {
    if (gameState.playerState != GameState::unstiff)
      setToUnstiff("not operational");
  }

  updateBallBuffer(gameState);
  updateIllegalMotionInSetTimestamps();
  updateIllegalPosition();

  // If this player is unstiff or in calibration mode, the game state stays reset and GameController messages are ignored.
  if (gameState.playerState == GameState::unstiff || gameState.playerState == GameState::prepare ||
      gameState.playerState == GameState::calibration)
    return;

  TLOGV(tlogger, "beyond unstiff/prepare/calib check");

  // at this point, we want to track and further changes to gameState or playerState for text logging
  GameState::State prevState = gameState.state;
  GameState::PlayerState prevPlayerState = gameState.playerState;


  // Update whether the GameController is "active".
  const bool useGameControllerData = theGameControllerData.timeLastPacketReceived > timeOfLastIntegratedGameControllerData;
  if(useGameControllerData)
    gameState.gameControllerActive = true;
  else if(theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) >= gameControllerTimeout)
    gameState.gameControllerActive = false;


  // Reset whistle state transitions that turn out to be wrong.
  if (gameStateOverridden)
  {
    switch (gameState.state)
    {
    case GameState::setupOwnKickOff:
    case GameState::setupOpponentKickOff:
    case GameState::setupDropBall:
      // A guessed READY state (after a goal) is reverted if
      // - sufficient time has elapsed without the GameController state actually changing to READY or
      // - the GameController continues to announce a set play.
      // TODO: Instead of checking getTimeSince(timeWhenStateStarted), we should check the time since
      //       the last whistle which would have made us switch to READY, because it could be that the
      //       *original* whistle which triggered the transition to READY was wrong, but a *later* whistle
      //       was true.
      if (gameState.gameControllerActive && theGameControllerData.state != STATE_READY &&
          !gameState.kickOffSetupFromTouchlines &&
          (gameState.getStateDuration() > goalSignalDelay + gameControllerOperatorDelay ||
           (theGameControllerData.setPlay != SET_PLAY_NONE &&
            gameState.getStateDuration() > gameControllerOperatorDelay)))
      {
        // Even if the game state had been overridden when the (wrong, as it turns out) transition to
        // READY was triggered, that should not be the case anymore because
        // - in the case of "sufficient time", that state would be sent truthfully now, too (as long as goalSignalDelay >= playingSignalDelay), and
        // - in case a set play is sent, it is true information.
        gameStateOverridden = false;
        minWhistleTimestamp = std::max(gameState.timeWhenStateStarted, theFrameInfo.time - acceptPastWhistleDelay);
        setGameState(convertGameControllerDataToState(theGameControllerData));
        gameState.timeWhenStateStarted =
            timeWhenStateStartedBeforeWhistle; // TODO: We could be in a completely different state now than we were when we started guessing.
        // gameState.timeWhenStateEnds = 0; // TODO: We could be in a state that requires this to be != 0.
      }
      break;
    case GameState::ownKickOff:
    case GameState::opponentKickOff:
    case GameState::ownPenaltyKick:
    case GameState::opponentPenaltyKick:
    case GameState::ownPenaltyShot:
    case GameState::opponentPenaltyShot:
    case GameState::playing:
      // A guessed PLAYING state is reverted if a player has been penalized for illegal motion in SET.
      // We do not include a timeout here because as soon as a robot moves, it is penalized anyway.
      if (gameState.gameControllerActive && theGameControllerData.state == STATE_SET &&
          checkForIllegalMotionInSetPenalty(timeWhenSwitchedToPlayingViaWhistle))
      {
        gameStateOverridden = false;
        minWhistleTimestamp = theFrameInfo.time;
        setGameState(convertGameControllerDataToState(theGameControllerData));
        gameState.timeWhenStateStarted = timeWhenStateStartedBeforeWhistle;
      }

      // we guess that we're in playing (after a free) if the ball has moved.
      // Such a guessed PLAYING state is reverted if a player has been penalized
      // for illegalPosition or the ball is returned to the legal start position for the free
      // or if the GC operator should by now have signalled that the free kick is over
      // but they have not.
      auto gameControllerState = convertGameControllerDataToState(theGameControllerData);
      auto recentIllegalPosition = [](const std::vector<unsigned> &occurrences, unsigned timestamp) -> bool
      { return *std::max_element(occurrences.begin(), occurrences.end()) > timestamp; };

      bool anyIllegalPosition =
          GameState::isFreeKick(gameControllerState) &&
          ((recentIllegalPosition(illegalPositionTimestampsOwnTeam, gameState.timeWhenStateStarted) &&
            GameState::isForOpponentTeam(gameControllerState)) ||
           (recentIllegalPosition(illegalPositionTimestampsOpponentTeam, gameState.timeWhenStateStarted) &&
            GameState::isForOwnTeam(gameControllerState)));
      bool ballMovedBackToFreeKickPosition =
          GameState::isConstrainedFreeKick(gameControllerState) &&
          checkBallLegal(gameControllerState, confidenceIntervalToLegalPositionNotFree);

      if (anyIllegalPosition || ballMovedBackToFreeKickPosition ||
          (gameState.getStateDuration() > gameControllerOperatorDelay))
      {
        gameStateOverridden = false;
        setGameState(gameControllerState);
        gameState.timeWhenStateStarted = timeWhenStateStartedBeforeBallMoved;
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + freeKickDuration;
      }

      // when doing a penalty kick, we might guess PLAYING based on a whistle.
      // Such a guessed PLAYING state is reverted to penaltyKick if any player is penalized for illegalPosition.
      // The gameControllerOperatorDelay is not considered, because the game controller might still send SET to hide the whistle
      if ((GameState::isPenaltyKick(gameControllerState) && !GameState::isReady(gameControllerState)) &&
          anyIllegalPosition)
      {
        setGameState(GameState::isForOwnTeam(gameControllerState) ? GameState::ownPenaltyKick : GameState::opponentPenaltyKick);
        gameState.timeWhenStateStarted = timeWhenStateStartedBeforeBallMoved;
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
      }
      break;
    }

    if (gameState.state != prevState)
      TLOGI_ANNOTATION(tlogger, "GameState ==> {} (was previously overridden {})",
                       TypeRegistry::getEnumName(gameState.state), TypeRegistry::getEnumName(prevState));
  }

  // A kick-off or penalty-kick is over once either the ball has moved, is no longer on a legal position or the time has elapsed.
  if ((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff ||
       gameState.state == GameState::ownPenaltyKick || gameState.state == GameState::opponentPenaltyKick) &&
      (checkBallHasMoved(gameState) || theFrameInfo.time >= gameState.timeWhenStateEnds ||
       !checkBallLegal(gameState.state, confidenceIntervalToLegalPositionFree)))
  {
    gameStateOverridden = true;
    timeWhenStateStartedBeforeBallMoved = gameState.timeWhenStateStarted;
    setGameState(GameState::playing);

    TLOGI_ANNOTATION(tlogger, "GameState ==> {} (was previously {})",
                     TypeRegistry::getEnumName(gameState.state), TypeRegistry::getEnumName(prevState));
  }

  // State transitions that are triggered by the whistle:
  if (!gameState.gameControllerActive || !theGameControllerData.isTrueData)
  {
    switch (gameState.state)
    {
    case GameState::opponentDirectFreeKick:
    case GameState::opponentIndirectFreeKick:
    // case GameState::opponentPushingFreeKick:
    case GameState::opponentThrowIn:
    case GameState::opponentGoalKick:
    case GameState::opponentCornerKick:

    case GameState::ownDirectFreeKick:
    case GameState::ownIndirectFreeKick:
    // case GameState::ownPushingFreeKick:
    case GameState::ownThrowIn:
    case GameState::ownGoalKick:
    case GameState::ownCornerKick:
      // Check for ball free
      if (checkBallHasMoved(gameState) || !checkBallLegal(gameState.state, confidenceIntervalToLegalPositionFree))
      {
        gameStateOverridden = true;
        timeWhenStateStartedBeforeBallMoved = gameState.timeWhenStateStarted;
        setGameState(GameState::playing);
      }
      [[fallthrough]];
    case GameState::playing:
    case GameState::ownKickOff:
    case GameState::opponentKickOff:
    case GameState::ownPenaltyKick:
    case GameState::opponentPenaltyKick:
      // The referee signals the transition to READY after a goal with a whistle.
      // There is a "cooldown period" in which whistles are ignored to avoid confusion with the whistle that started PLAYING.
      // Since goals from penalty kicks can be scored quicker, the have a different (shorter) cooldown period.
      if (checkWhistleForGoal &&
          checkForWhistle(std::max(minWhistleTimestamp + ((gameState.state == GameState::ownPenaltyKick ||
                                                           gameState.state == GameState::opponentPenaltyKick)
                                                              ? ignoreWhistleAfterPenaltyKick
                                                              : ignoreWhistleAfterKickOff),
                                   theBallInGoal.lastTimeInGoal > acceptWhistleBeforeGoal
                                       ? theBallInGoal.lastTimeInGoal - acceptWhistleBeforeGoal
                                       : 0u),
                          gameState.gameControllerActive) &&
          (theFrameInfo.getTimeSince(theBallInGoal.lastTimeInGoal) <= acceptBallInGoalDelay || !checkBallForGoal))
      {
        // Remember start of previous state in case the whistle detection turns out to be wrong.
        gameStateOverridden = true;
        timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
        gameState.kickOffSetupFromTouchlines = false;
        setGameState(theBallInGoal.inOwnGoal ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff);
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
        ++(theBallInGoal.inOwnGoal ? gameState.opponentTeam : gameState.ownTeam).score;
        gameState.whistled = true;
      }
      break;

    case GameState::waitForOwnKickOff:
    case GameState::waitForOpponentKickOff:
    case GameState::waitForDropBall:
    case GameState::waitForOwnPenaltyKick:
    case GameState::waitForOpponentPenaltyKick:
    case GameState::waitForOwnPenaltyShot:
    case GameState::waitForOpponentPenaltyShot:
      if (checkForWhistle(minWhistleTimestamp, gameState.gameControllerActive))
      {
        // Remember start of previous state in case the whistle detection turns out to be wrong.
        gameStateOverridden = true;
        timeWhenStateStartedBeforeWhistle = gameState.timeWhenStateStarted;
        timeWhenSwitchedToPlayingViaWhistle = theFrameInfo.time;
        minWhistleTimestamp = theFrameInfo.time;
        switch (gameState.state)
        {
        case GameState::waitForOwnKickOff: setGameState(GameState::ownKickOff); break;
        case GameState::waitForOpponentKickOff: setGameState(GameState::opponentKickOff); break;
        case GameState::waitForDropBall: setGameState(GameState::playing); break;
        case GameState::waitForOwnPenaltyKick: setGameState(GameState::ownPenaltyKick); break;
        case GameState::waitForOpponentPenaltyKick: setGameState(GameState::opponentPenaltyKick); break;
        case GameState::waitForOwnPenaltyShot: setGameState(GameState::ownPenaltyShot); break;
        case GameState::waitForOpponentPenaltyShot: setGameState(GameState::opponentPenaltyShot); break;
        }
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + (gameState.isKickOff() ? kickOffDuration
                                                                        : gameState.isPenaltyKick() ? penaltyKickDuration
                                                                                                    : 0);
        gameState.whistled = true;
      }
      break;
    }

    if (gameState.state != prevState)
      TLOGI_ANNOTATION(tlogger, "GameState ==> {}{} based on whistle etc (was previously {})",
                       TypeRegistry::getEnumName(gameState.state), gameStateOverridden ? " [overridden]" : "",
                       TypeRegistry::getEnumName(prevState));
  }

  // Transition from Standby to Ready
//   if (gameState.state == GameState::standby && checkForRefereeSignal(gameState, RefereeSignal::ready))
//   {
//     const bool isDropBall = theGameControllerData.kickingTeam == KICKING_TEAM_NONE;
//     const bool isKickingTeam = theGameControllerData.isOwnTeamKick();
//     gameState.state = isDropBall ? GameState::setupDropBall
//                       : isKickingTeam ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
//     gameState.timeWhenStateStarted = theFrameInfo.time;
//     gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
//     gameState.kickOffSetupFromTouchlines = true;
//     gameStateOverridden = true;
// 
//     TLOGI_ANNOTATION(tlogger, "GameState ==> {} based on referee signal (was previously {})",
//                      TypeRegistry::getEnumName(gameState.state), TypeRegistry::getEnumName(prevState));
//   }

  if (useGameControllerData)
  {
    // gameState.competitionPhase = static_cast<GameState::CompetitionPhase>(theGameControllerData.competitionPhase);
    // gameState.competitionPhase = GameState::playOff; // penalty shootout possible, clock affected, but trust GC anyway

    // The index of the own team within theGameControllerData.
    const std::size_t ownTeamIndex = theGameControllerData.ownTeamIndex();
    gameState.leftHandTeam = ownTeamIndex == 0;

    // Everything else would be a bug in the GameControllerDataProvider:
    ASSERT(theGameControllerData.ownTeamInfo().teamNumber == Global::getSettings().teamNumber);

    // The game phase is always reported truthfully by the GameController.
    // See below for timeWhenPhaseEnds.
    gameState.phase = convertGameControllerDataToPhase(theGameControllerData);

    const auto gameControllerState = convertGameControllerDataToState(theGameControllerData);

    if (gameStateOverridden)
    {
      // No need to override if the GC state matches what the guessed state
      if (gameState.state == gameControllerState)
        gameStateOverridden = false;
      // We only override during SET (for guessed playing), PLAYING (for guessed goal) or STANDBY (for guessed ready)
      else if (!GameState::isSet(gameControllerState) && !GameState::isPlaying(gameControllerState) &&
               gameControllerState != GameState::standby)
        gameStateOverridden = false;
      // Don't override a SET state that was reached from READY
      else if (GameState::isSet(gameControllerState) && gameState.isReady())
        gameStateOverridden = false;
      // Cancel override if the free kick situation changed
      else if (GameState::isFreeKick(gameControllerState) != GameState::isFreeKick(lastGameControllerState) ||
               (GameState::isFreeKick(gameControllerState) && lastGameControllerState != gameControllerState))
        gameStateOverridden = false;
    }

    if (!gameStateOverridden && gameControllerState != gameState.state &&
        ((!gameState.isFreeKick() && !gameState.isPenaltyKick()) || gameControllerState != gameState.stateForOtherTeam()))
    {
      // Ignore older whistles when switching to some states.
      if((!gameState.isSet() && GameState::isSet(gameControllerState)) ||
         (!gameState.isPlaying() && GameState::isPlaying(gameControllerState)))
        minWhistleTimestamp = theFrameInfo.time;

      // kickOffSetupFromTouchlines: We are switching from some INITIAL state to READY and we already integrated a GameController packet recently.
      if ((gameControllerState == GameState::setupOwnKickOff ||
           gameControllerState == GameState::setupOpponentKickOff || gameControllerState == GameState::setupDropBall) &&
          gameState.isInitial() &&
          theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) < gameControllerTimeout)
        gameState.kickOffSetupFromTouchlines = true;

      // Actually set the new state and associated timestamps:
      setGameState(gameControllerState);

      if (gameState.state != prevState)
        TLOGI_ANNOTATION(tlogger, "GameState ==> {} based on Game Controller (was previously {})",
                         TypeRegistry::getEnumName(gameState.state), TypeRegistry::getEnumName(prevState));

      switch (gameControllerState)
      {
      case GameState::setupOwnKickOff:
      case GameState::setupOpponentKickOff:
      case GameState::setupDropBall:
        // Switching to READY while changing the score means we haven't heard the whistle after a goal
        // or switching to READY when the KickOff-Setup starts from the touchline means we haven't detected the referee signal.
        // In those cases, the state actually began earlier.
        if (!theGameControllerData.isTrueData &&
            ((theGameControllerData.ownTeamInfo().score != gameState.ownTeam.score ||
              theGameControllerData.opponentTeamInfo().score != gameState.opponentTeam.score) ||
             gameState.kickOffSetupFromTouchlines))
          gameState.timeWhenStateStarted -= (kickOffSetupDuration - theGameControllerData.secondaryTime * 1000);

        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffSetupDuration;
        break;

      case GameState::ownKickOff:
      case GameState::opponentKickOff:
        if (!theGameControllerData.isTrueData)
          gameState.timeWhenStateStarted -= (kickOffDuration - theGameControllerData.secondaryTime * 1000);

        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + kickOffDuration;
        break;

      case GameState::setupOwnPenaltyKick:
      case GameState::setupOpponentPenaltyKick:
        // The GameController never delays this state change.
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickSetupDuration;
        break;

      case GameState::ownPenaltyKick:
      case GameState::opponentPenaltyKick:
      case GameState::ownPenaltyShot:
      case GameState::opponentPenaltyShot:
        // Penalty kicks (whether in a game or a penalty shoot-out) are delayed by the GameController.
        if (!theGameControllerData.isTrueData)
          gameState.timeWhenStateStarted -= playingSignalDelay;

        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + penaltyKickDuration;
        break;

      case GameState::ownDirectFreeKick:
      case GameState::opponentDirectFreeKick:
      case GameState::ownIndirectFreeKick:
      case GameState::opponentIndirectFreeKick:
      case GameState::ownThrowIn:
      case GameState::opponentThrowIn:
      case GameState::ownGoalKick:
      case GameState::opponentGoalKick:
      case GameState::ownCornerKick:
      case GameState::opponentCornerKick:
        gameState.timeWhenStateEnds = gameState.timeWhenStateStarted + freeKickDuration;
        break;
      default:
        // All other states don't have a fixed duration.
        gameState.timeWhenStateEnds = 0;
        break;
      }
    }

    const auto fillTeam = [&](const RoboCup::TeamInfo &teamInfo, bool speculatingGoalFor, GameState::Team &team)
    {
      static_assert(std::tuple_size<decltype(team.playerStates)>::value == MAX_NUM_PLAYERS);
      for(std::size_t i = 0; i < team.playerStates.size(); ++i)
      {
        const GameState::PlayerState playerState = convertPenaltyToPlayerState(teamInfo.players[i].penalty);
        if(playerState != team.playerStates[i])
          team.timeWhenPlayerStatesStarted[i] = theFrameInfo.time;
        team.playerStates[i] = playerState;
      }
      team.number = teamInfo.teamNumber;
      team.fieldPlayerColor = static_cast<GameState::Team::Color>(teamInfo.fieldPlayerColor);
      team.goalkeeperColor = static_cast<GameState::Team::Color>(teamInfo.goalkeeperColor);
      team.goalkeeperNumber = Settings::toPlayerNumber(teamInfo.goalkeeper);
      team.score = teamInfo.score + (speculatingGoalFor ? 1 : 0);
      team.messageBudget = teamInfo.messageBudget;
    };

    // if we're in a (non-drop-ball) kickoff state, then somebody likely scored
    fillTeam(theGameControllerData.ownTeamInfo(),
             gameState.state == GameState::setupOpponentKickOff && gameControllerState != gameState.state,
             gameState.ownTeam);
    fillTeam(theGameControllerData.opponentTeamInfo(),
             gameState.state == GameState::setupOwnKickOff && gameControllerState != gameState.state,
             gameState.opponentTeam);

    const auto gameControllerPlayerState = convertPenaltyToPlayerState(theGameControllerData.ownPlayerInfo().penalty);

    if (gameControllerPlayerState != gameState.playerState)
    {
      setPlayerState(gameControllerPlayerState);

      TLOGI_ANNOTATION(tlogger, "PlayerState ==> {} based on Game Controller (was previously {})",
                        TypeRegistry::getEnumName(gameState.playerState), TypeRegistry::getEnumName(prevPlayerState));

      // If this is the first GameController packet for some time, it might be that the robot actually has had the state for a longer time.
      if(theFrameInfo.getTimeSince(timeOfLastIntegratedGameControllerData) >= gameControllerTimeout)
        gameState.timeWhenPlayerStateStarted = 0;
      else
        gameState.timeWhenPlayerStateStarted = theFrameInfo.time;
    }

    gameState.timeWhenPenaltyEnds = gameState.isPenalized()
                                    ? theGameControllerData.timeLastPacketReceived
                                      + theGameControllerData.ownPlayerInfo().secsTillUnpenalised * 1000
                                    : 0;

    timeOfLastIntegratedGameControllerData = theGameControllerData.timeLastPacketReceived;
    lastGameControllerState = gameControllerState;
  }
  else if (!gameState.gameControllerActive)
  {
    // Manual HRI (only when we haven't received from a GameController recently):
    if (manualGameStateChangeRequested)
    {
      if (gameState.playerState == GameState::active)
        setPlayerState(GameState::penalizedManual);
      else
      {
        if (gameState.phase == GameState::penaltyShootout)
        {
          // When a penalty shoot-out is done using the button interface (which cannot
          // happen in regular games, since the GameController *must* be used there),
          // the state never switches to afterPenaltyShot/waitForPenaltyShot. However,
          // the SelfLocator handles this as it also resets the pose when the robot is
          // unpenalized (the SideInformationProvider doesn't, but its output isn't
          // used during penalty shoot-outs).
          setGameState(manualPenaltyShootoutForOwnTeam ? GameState::ownPenaltyShot : GameState::opponentPenaltyShot);
          gameState.timeWhenStateStarted = theFrameInfo.time - playingSignalDelay;
          gameState.timeWhenPhaseEnds = gameState.timeWhenStateEnds =
              gameState.timeWhenStateStarted + penaltyKickDuration;
        }
        else
        {
          minWhistleTimestamp = theFrameInfo.time;
          setGameState(GameState::playing);
        }
        setPlayerState(GameState::active);
      }
    }

    if (gameState.isInitial() && gameState.playerState == GameState::active)
    {
      if (manualPhaseChangeRequested)
      {
        if(gameState.phase != GameState::penaltyShootout)
        {
          gameState.phase = GameState::penaltyShootout;
          gameState.timeWhenPhaseEnds = 0;
          setGameState(GameState::beforePenaltyShootout);
          manualPenaltyShootoutForOwnTeam = true;
          SystemCall::say("Penalty striker");
        }
        else if(manualPenaltyShootoutForOwnTeam)
        {
          manualPenaltyShootoutForOwnTeam = false;
          SystemCall::say("Penalty keeper");
        }
        else
        {
          gameState.phase = GameState::firstHalf;
          gameState.timeWhenPhaseEnds = 0;
          setGameState(GameState::beforeHalf);
          SystemCall::say("Soccer");
        }
      }
    }

    if (gameState.state != prevState)
      TLOGI_ANNOTATION(tlogger, "GameState ==> {} based on button interface (was previously {})",
                       TypeRegistry::getEnumName(gameState.state), TypeRegistry::getEnumName(prevState));
    
    if (gameState.playerState != prevPlayerState)
      TLOGI_ANNOTATION(tlogger, "PlayerState ==> {} based on button interfaces (was previously {})",
                       TypeRegistry::getEnumName(gameState.playerState), TypeRegistry::getEnumName(prevPlayerState));
  }

  if(gameState.gameControllerActive && !gameStateOverridden)
  {
    // This is done this way so that old code that was using secsRemaining still works:
    gameState.timeWhenPhaseEnds = theFrameInfo.time + theGameControllerData.secsRemaining * 1000;
  }

  if (gameState.state != GameState::setupOwnKickOff && gameState.state != GameState::setupOpponentKickOff &&
      gameState.state != GameState::setupDropBall)
  {
    gameState.kickOffSetupFromTouchlines = false;
  }

  gameState.kickingTeamKnown = !theGameControllerData.isNoTeamKick();
  // updateKickingTeamByRefereeSignal(gameState);
  // updateKickingTeamByIllegalPosition(gameState);
  // updatePenalizedForPushing();
}

void GameStateProvider::reset(GameState& gameState)
{
  gameState = GameState();

  gameStateOverridden = false;
  manualPenaltyShootoutForOwnTeam = false;
  timeWhenStateNotAfterHalf = theFrameInfo.time;
  ballPositions.clear();
  lastBallAddedToBuffer = 0;
  illegalMotionInSetTimestamps.clear();
  illegalPositionTimestampsOwnTeam.clear();
  illegalPositionTimestampsOpponentTeam.clear();
  timeOfLastIntegratedGameControllerData = 0;
  timeWhenStateStartedBeforeWhistle = 0;
  timeWhenSwitchedToPlayingViaWhistle = 0;
  minWhistleTimestamp = 0;

  timeWhenStopActivated = 0;
  prevStoppedTimeInCurrentState = 0;
  prevStoppedTimeInCurrentPlayerState = 0;
}


void GameStateProvider::updateFromNaoHri(GameState& gameState)
{
  unstiffModeRequested =
      theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > unstiffHeadButtonPressDuration &&
      (theEnhancedKeyStates.pressedDuration[KeyStates::headMiddle] > unstiffHeadButtonPressDuration ||
       theEnhancedKeyStates.pressedDuration[KeyStates::chest] > unstiffHeadButtonPressDuration) &&
      theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > unstiffHeadButtonPressDuration;

  activeModeRequested = false;
  calibrationModeRequested = false;
  manualGameStateChangeRequested = false;
  manualPhaseChangeRequested = false;

  // Nao chest button

  switch (gameState.playerState)
  {
  case GameState::unstiff:
    if (theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      activeModeRequested = true;
    break;
  case GameState::calibration:
    break; // do nothing
  default:
    if (gameState.isInitial() && (gameState.playerState == GameState::active) &&
             theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > calibrationHeadButtonPressDuration &&
             theEnhancedKeyStates.isPressedFor(KeyStates::chest, 1000u))
      calibrationModeRequested = true;
    else if (theEnhancedKeyStates.hitStreak[KeyStates::chest] == 1)
      manualGameStateChangeRequested = true;
  }

  // Nao foot bumpers
  if (theEnhancedKeyStates.hitStreak[KeyStates::rFootRight] == 1)
    manualPhaseChangeRequested = true;
}

void GameStateProvider::updateFromBoosterK1Hri([[maybe_unused]] GameState& gameState)
{
  unstiffModeRequested = false; // handled outside game state
  prepareModeRequested = false;
  activeModeRequested = false; // handled outside game state
  calibrationModeRequested = false; // TODO - gamePad?
  manualGameStateChangeRequested = false;
  manualPhaseChangeRequested = false; // TODO - gamePad or not used

#ifdef TARGET_BOOSTER
  // tmp debug
  // if (theEnhancedKeyStates.hitStreak[KeyStates::k1_stand] == 1)
  //   TLOGI(tlogger, "STAND button clicked once");
  // else if (theEnhancedKeyStates.getHitStreakFor(KeyStates::k1_stand, 3000u, 50u) == 1)
  //   TLOGI(tlogger, "STAND button long clicked once and timeSinceOperational is {}",
  //         theFrameInfo.getTimeSince(timeWhenRobotBecameOperational));
  // else if (theEnhancedKeyStates.hitStreak[KeyStates::k1_walk] == 1)
  //   TLOGI(tlogger, "WALK button clicked once");

  constexpr unsigned fromSafetyTrippedMs = 2000u;
  constexpr unsigned toCalibrationMs = 2000u;

  // buttons only active if we're operational
  if (theRobotSafetyState.state == RobotSafetyState::operational)
  {
    // if (theEnhancedKeyStates.clickedDuration[KeyStates::k1_f1] >= fromSafetyTrippedMs)
    //   TLOGD(tlogger, "detected F1 button long-clicked >{} ms once", fromSafetyTrippedMs);
    // else if (theEnhancedKeyStates.clickedDuration[KeyStates::k1_stand] >= fromSafetyTrippedMs)
    //   TLOGD(tlogger, "detected STAND button long-clicked >{} ms once", fromSafetyTrippedMs);

    bool nearSwitchToOperational = theFrameInfo.getTimeSince(timeWhenRobotBecameOperational) < 300;

    // F1 is an e-stop: it must work from every playerState, unconditionally, overriding whatever
    // the GameController is saying. Checked here, before the per-state switch below, rather than
    // buried in a single case, so it can never be missed for a state that doesn't explicitly
    // handle it (this used to only do anything while already in GameState::unstiff).
    if (theEnhancedKeyStates.clickedDuration[KeyStates::k1_f1] >= fromSafetyTrippedMs)
    {
      TLOGI(tlogger, "F1 button long-clicked: unstiffModeRequested (e-stop)");
      unstiffModeRequested = true;
    }

    switch (gameState.playerState)
    {
    case GameState::unstiff:
      if (!nearSwitchToOperational &&
          (theEnhancedKeyStates.hitStreak[KeyStates::k1_stand] == 1))
      {
        TLOGI(tlogger, "Unstiff: STAND button clicked once: prepareModeRequested");
        prepareModeRequested = true;
      }
      else if (nearSwitchToOperational && (theEnhancedKeyStates.clickedDuration[KeyStates::k1_stand] >= fromSafetyTrippedMs))
      {
        TLOGI(tlogger, "Unstiff: STAND button long-clicked once near switching to operational: prepareModeRequested");
        prepareModeRequested = true;
      }
      break;

    case GameState::prepare:
      if (theEnhancedKeyStates.hitStreak[KeyStates::k1_walk] == 1)
      {
        TLOGI(tlogger, "Prepare: WALK button clicked once: activeModeRequested");
        activeModeRequested = true;
      }
      break;

    case GameState::calibration:
      if (theEnhancedKeyStates.hitStreak[KeyStates::k1_stand] == 1)
      {
        TLOGI(tlogger, "calibration: STAND button clicked once: prepareModeRequested");
        prepareModeRequested = true;
      }
      break;

    default:
      if (gameState.isInitial() && (gameState.playerState == GameState::active) &&
          (theEnhancedKeyStates.pressedDuration[KeyStates::k1_walk] > toCalibrationMs))
      {
        TLOGI(tlogger, "Active: WALK button long pressed: calibrationModeRequested");
        calibrationModeRequested = true;
      }
      else if (theEnhancedKeyStates.hitStreak[KeyStates::k1_walk] == 1)
      {
        TLOGI(tlogger, "Active: WALK button clicked: manualGameStateChangeRequested");
        manualGameStateChangeRequested = true;
      }
      else if (theEnhancedKeyStates.hitStreak[KeyStates::k1_stand] == 1)
      {
        TLOGI(tlogger, "Active: STAND button clicked: prepareModeRequested");
        prepareModeRequested = true;
      }
      else // buttons (above) take precedence over any game pad control (in here)
      {
        // TODO
      }
    }
  }
#endif // TARGET_BOOSTER
}

bool GameStateProvider::checkForWhistle(unsigned from, bool useGameControllerData) const
{
  std::vector<const Whistle*> data;
  int numOfChannels = 0;
  int numOfMissingChannels = 0;
  size_t numOfVoters = 0;

  if(theWhistle.channelsUsedForWhistleDetection > 0)
  {
    ++numOfVoters;
    numOfChannels += theWhistle.channelsUsedForWhistleDetection;
    if(theWhistle.lastTimeWhistleDetected > from)
      data.emplace_back(&theWhistle);
  }

  for(const Teammate& teammate : theTeamData.teammates)
    if(teammate.theWhistle.channelsUsedForWhistleDetection > 0)
    {
      ++numOfVoters;
      if(teammate.theWhistle.lastTimeWhistleDetected > from)
      {
        numOfChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
        data.emplace_back(&teammate.theWhistle);
      }
      else
        numOfMissingChannels += teammate.theWhistle.channelsUsedForWhistleDetection;
    }

  if(useGameControllerData)
  {
    size_t numOfPlayers = 0;
    const RoboCup::TeamInfo& ownTeam = theGameControllerData.ownTeamInfo();
    for(int i = 0; i < MAX_NUM_PLAYERS; ++i)
      if(ownTeam.players[i].penalty == PENALTY_NONE)
        ++numOfPlayers;
    const size_t numOfMissingPlayers = numOfPlayers - std::min(numOfPlayers, theTeamData.teammates.size() + 1);
    numOfVoters += numOfMissingPlayers;
    numOfMissingChannels += static_cast<int>(2 * numOfMissingPlayers); // Assume that unknown teammates use 2 channels
  }

  if(data.size() < minVotersForWhistle && data.size() < numOfVoters)
  {
    const size_t missing = minVotersForWhistle - data.size();
    numOfChannels += static_cast<int>(missing * numOfMissingChannels / (numOfVoters - data.size())); // Approximate number of missing channels
  }

  std::sort(data.begin(), data.end(),
            [](const Whistle *w1, const Whistle *w2) -> bool
            { return w1->lastTimeWhistleDetected < w2->lastTimeWhistleDetected; });

  for(size_t i = 0; i < data.size(); ++i)
  {
    float totalConfidence = 0.f;
    for(size_t j = i; j < data.size(); ++j)
    {
      if(static_cast<int>(data[j]->lastTimeWhistleDetected - data[i]->lastTimeWhistleDetected) > maxWhistleTimeDifference)
        break;

      totalConfidence += data[j]->confidenceOfLastWhistleDetection
                         * static_cast<float>(data[j]->channelsUsedForWhistleDetection);

      if(totalConfidence / numOfChannels > minWhistleAverageConfidence)
        return true;
    }
  }

  return false;
}

bool GameStateProvider::checkBallHasMoved(const GameState& gameState) const
{
  // Did anyone kick the ball?
  if(gameState.isForOwnTeam() && (theMotionInfo.lastKickTimestamp > gameState.timeWhenStateStarted + minOwnFreeKickDelay ||
     std::any_of(theTeamData.teammates.begin(), theTeamData.teammates.end(),
                 [&](const Teammate& teammate)
                 { return teammate.theIndirectKick.lastKickTimestamp > gameState.timeWhenStateStarted + minOwnFreeKickDelay; })))
    return true;

  // Is the ball (significantly) closer to me than it could be, given that I am legally positioned?
  // -> theBallModel.timeWhenLastSeen == theFrameInfo.time can't be used because the ball model is (or at least can be) from the previous frame (USES).
  if(gameState.state == GameState::opponentKickOff &&
     theBallModel.timeWhenLastSeen > std::max(theFrameInfo.time - 500, gameState.timeWhenStateStarted) &&
     theBallModel.lastPerception.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - ballHasMovedCloseToRobotThreshold))
    return true;

  if((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff) &&
     ballOutOfCenterCircleCounter >= ballOutOfCenterCircleCounterThreshold)
    return true;

  // Is there sufficient difference in recent (median filtered) ball positions?
  if(ballPositions.size() >= 6)
  {
    const auto medianOfThree = [](const Vector2f& a, const Vector2f& b, const Vector2f& c) -> const Vector2f&
    {
      const float aN = a.squaredNorm();
      const float bN = b.squaredNorm();
      const float cN = c.squaredNorm();

      if(aN >= bN)
      {
        if(cN >= aN)
          return a;
        else // c < a
          return bN >= cN ? b : c;
      }
      else // b > a
      {
        if(cN >= bN)
          return b;
        else // c < b
          return aN >= cN ? a : c;
      }
    };
    const std::size_t firstIndex = ballPositions.size() - 1;
    const Vector2f& start = medianOfThree(ballPositions.back(), ballPositions[firstIndex - 1], ballPositions[firstIndex - 2]);
    const Vector2f& end = medianOfThree(ballPositions.front(), ballPositions[1], ballPositions[2]);
    if((start - end).squaredNorm() > sqr(ballHasMovedTolerance) * std::max(1.f, std::min(start.squaredNorm(), end.squaredNorm()) / sqr(1000.f)))
      return true;
  }
  return false;
}

bool GameStateProvider::checkBallLegal(const GameState::State& gameState, const float confidenceIntervalToLegalPosition) const
{
  // only in these cases the ball position is constrained (kickoff as well but the above code deals with that case)
  bool ballPositionConstrained = GameState::isThrowIn(gameState) || GameState::isGoalKick(gameState) ||
                                 GameState::isCornerKick(gameState) || (gameState == GameState::ownPenaltyKick) ||
                                 (gameState == GameState::opponentPenaltyKick);

  if ((theBallModel.timeWhenLastSeen > theFrameInfo.time - 500) && ballPositionConstrained)
  {
    const Vector2f globalBall = theRobotPose * theBallModel.estimate.position;
    // compute the covariance of the global ball model
    // start with the covariance of the local model
    Matrix2f cov = theBallModel.estimate.covariance;

    //rotate covariance matrix
    cov = Covariance::rotateCovarianceMatrix(cov, theRobotPose.rotation);

    // add covariance of the localization
    // TODO: think about how to incorporate rotational uncertainty
    cov += theRobotPose.covariance.block<2, 2>(0, 0);

    COVARIANCE_ELLIPSES_2D("module:GameStateProvider:ballCovariance", cov, globalBall);

    //get the direction to the nearest legal position
    Vector2f direction(0.f, 0.f);

    if(GameState::isThrowIn(gameState))
      direction << 0.f, (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
    else if(GameState::isGoalKick(gameState))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftGoalArea : theFieldDimensions.yPosRightGoalArea) - globalBall.y();
      if(GameState::isForOpponentTeam(gameState))
        direction.x() = theFieldDimensions.xPosOpponentGoalArea - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOwnGoalArea - globalBall.x();
    }
    else if(GameState::isCornerKick(gameState))
    {
      direction.y() = (globalBall.y() > 0 ? theFieldDimensions.yPosLeftTouchline : theFieldDimensions.yPosRightTouchline) - globalBall.y();
      if(GameState::isForOpponentTeam(gameState))
        direction.x() = theFieldDimensions.xPosOwnGoalLine - globalBall.x();
      else
        direction.x() = theFieldDimensions.xPosOpponentGoalLine - globalBall.x();
    }
    else if(gameState == GameState::ownPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0) - globalBall;
    }
    else if(gameState == GameState::opponentPenaltyKick)
    {
      direction = Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0) - globalBall;
    }
    // ball is exactly on a legal position
    if(direction.squaredNorm() == 0.f)
      return true;

    ARROW("module:GameStateProvider:ballCovariance", globalBall.x(), globalBall.y(),
          (globalBall + direction).x(), (globalBall + direction).y(), 10, Drawings::PenStyle::solidPen,
          ColorRGBA::red);

    // construct covariance ellipse
    float a, b, angle;
    Covariance::errorEllipse(cov, a, b, angle, confidenceIntervalToLegalPosition);

    // compute radius in the previous calculated direction
    Angle theta(angle);
    theta += direction.angle();
    theta.normalize();

    const float radius = (a * b) / std::sqrt(sqr(a * std::sin(theta)) + sqr(b * std::cos(theta)));

    //if the ball is further from the next legal position than the radius + a placement tolerance it is likely that the ball was already played
    if (sqr(radius + ballPlacementTolerance) < direction.squaredNorm())
      return false;
  }
  return true;
}

// bool GameStateProvider::checkForRefereeSignal(const GameState& gameState, const RefereeSignal::Signal signal) const
// {
//   unsigned timeWhenStateStarted = std::max(timeWhenKickingTeamWasOverridden, gameState.timeWhenStateStarted);
//   if(theRefereeSignal.signal == signal && theRefereeSignal.timeWhenDetected >= timeWhenStateStarted)
//     return true;
//   else
//     for(const Teammate& teammate : theTeamData.teammates)
//       if(teammate.theRefereeSignal.signal == signal && teammate.theRefereeSignal.timeWhenDetected >= timeWhenStateStarted)
//         return true;
//   return false;
// }

bool GameStateProvider::checkForIllegalMotionInSetPenalty(unsigned from) const
{
  return std::any_of(illegalMotionInSetTimestamps.begin(), illegalMotionInSetTimestamps.end(), [from](unsigned timestamp)
  {
    return timestamp > from;
  });
}

void GameStateProvider::updateBallBuffer(const GameState& gameState)
{
  // Only buffer balls during states in which the ball should be stationary.
  if(gameState.state != GameState::ownKickOff && gameState.state != GameState::opponentKickOff &&
     gameState.state != GameState::ownPenaltyKick && gameState.state != GameState::opponentPenaltyKick &&
     !gameState.isFreeKick())
  {
    ballPositions.clear();
    lastBallUsedForCenterCircleCounter = lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
    ballOutOfCenterCircleCounter = 0;
    return;
  }

  if ((gameState.state == GameState::ownKickOff || gameState.state == GameState::opponentKickOff) &&
      theBallModel.timeWhenLastSeen > lastBallUsedForCenterCircleCounter)
  {
    const Vector2f ballPosition = theRobotPose * theBallModel.estimate.position;
    const float ballOutsideCircleDist = theFieldDimensions.centerCircleRadius + theFieldDimensions.fieldLinesWidth * 0.5f +
            theBallSpecification.radius + ballOutOfCenterCircleTolerance;

    if (ballPosition.squaredNorm() >= sqr(ballOutsideCircleDist))
      ++ballOutOfCenterCircleCounter;
    else if(ballOutOfCenterCircleCounter)
      --ballOutOfCenterCircleCounter;

    lastBallUsedForCenterCircleCounter = theBallModel.timeWhenLastSeen + ballSaveInterval;
  }

  // Remove balls when the camera matrix is likely to be unstable.
  // If this should someday also work if the robot is walking, the ball buffer would need odometry updates.
  if (theMotionInfo.executedPhase != MotionPhase::stand ||
      theIMUValueState.gyroValues.stableSinceTimestamp == theIMUValueState.timestamp)
    ballPositions.clear();
  else if (theBallModel.timeWhenLastSeen > lastBallAddedToBuffer + ballSaveInterval)
  {
    // Only add new balls every ballSaveInterval milliseconds.
    ballPositions.push_front(theBallModel.lastPerception);
    lastBallAddedToBuffer = theBallModel.timeWhenLastSeen;
  }
}

void GameStateProvider::updateIllegalMotionInSetTimestamps()
{
  // For each player, set the timestamp in the first frame in which the penalty is illegal motion in set.
  illegalMotionInSetTimestamps.resize(NUM_TEAMS * MAX_NUM_PLAYERS, 0);
  for(unsigned int i = 0; i < NUM_TEAMS; ++i)
  {
    for (unsigned int j = 0; j < MAX_NUM_PLAYERS; ++j)
    {
      if (theGameControllerData.teams[i].players[j].penalty != PENALTY_MOTION_IN_SET)
        illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] = 0;
      else if (illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] == 0)
        illegalMotionInSetTimestamps[i * MAX_NUM_PLAYERS + j] = theFrameInfo.time;
    }
  }
}

void GameStateProvider::updateIllegalPosition()
{
  illegalPositionTimestampsOwnTeam.resize(MAX_NUM_PLAYERS, 0);
  illegalPositionTimestampsOpponentTeam.resize(MAX_NUM_PLAYERS, 0);

  const RoboCup::TeamInfo& ownTeam = theGameControllerData.ownTeamInfo();
  const RoboCup::TeamInfo& opponentTeam = theGameControllerData.opponentTeamInfo();
 
  for (unsigned int j = 0; j < MAX_NUM_PLAYERS; ++j)
  {
    if (ownTeam.players[j].penalty != PENALTY_ILLEGAL_POSITIONING)
      illegalPositionTimestampsOwnTeam[j] = 0;
    else if (illegalPositionTimestampsOwnTeam[j] == 0)
      illegalPositionTimestampsOwnTeam[j] = theFrameInfo.time;

    if (opponentTeam.players[j].penalty != PENALTY_ILLEGAL_POSITIONING)
      illegalPositionTimestampsOpponentTeam[j] = 0;
    else if (illegalPositionTimestampsOpponentTeam[j] == 0)
      illegalPositionTimestampsOpponentTeam[j] = theFrameInfo.time;
  }
}

// void GameStateProvider::updatePenalizedForPushing()
// {
//   penalizedForPushingOwnTeam.resize(MAX_NUM_PLAYERS, false);
//   unsigned ownTeam = theGameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;
//   for(unsigned int j = 0; j < MAX_NUM_PLAYERS; ++j)
//     penalizedForPushingOwnTeam[j] = theGameControllerData.teams[ownTeam].players[j].penalty == PENALTY_SPL_PLAYER_PUSHING;
// }
// 
// void GameStateProvider::updateKickingTeamForPushing(const GameControllerData& gameControllerData)
// {
//   isKickingTeam = true;
//   // Is there a new pushing penalty for a player in the own team?
//   const std::size_t ownTeamIndex = gameControllerData.teams[0].teamNumber == Global::getSettings().teamNumber ? 0 : 1;
//   for(std::size_t i = 0; i < MAX_NUM_PLAYERS; ++i)
//     if(gameControllerData.teams[ownTeamIndex].players[i].penalty == PENALTY_SPL_PLAYER_PUSHING && !penalizedForPushingOwnTeam[i])
//       isKickingTeam = false;
// }

// void GameStateProvider::updateKickingTeamByRefereeSignal(GameState& gameState)
// {
//   // Only execute during free kick when the kicking team is hidden.
//   if(gameState.kickingTeamKnown || !gameState.isThrowIn())
//     return;
// 
//   const bool forOwnTeam = checkForRefereeSignal(gameState, gameState.leftHandTeam ? RefereeSignal::kickInRight : RefereeSignal::kickInLeft);
//   const bool forOpponentTeam = checkForRefereeSignal(gameState, gameState.leftHandTeam ? RefereeSignal::kickInLeft : RefereeSignal::kickInRight);
//   if(forOwnTeam)
//   {
//     isKickingTeam = true;
//     gameState.state = GameState::ownKickIn;
//   }
//   if(forOpponentTeam) // or for both teams
//   {
//     isKickingTeam = false;
//     gameState.state = GameState::opponentKickIn;
//   }
// }

// void GameStateProvider::updateKickingTeamByIllegalPosition(GameState& gameState)
// {
//   // Only execute during free kick when the kicking team is hidden.
//   if(gameState.kickingTeamKnown || !gameState.isFreeKick())
//     return;
// 
//   // If we assume that the free kick is for the opponent, but they got an illegal position penalty,
//   // the free kick is for us.
//   if(gameState.isForOpponentTeam() &&
//      *std::max_element(illegalPositionTimestampsOpponentTeam.begin(), illegalPositionTimestampsOpponentTeam.end()) > gameState.timeWhenStateStarted)
//   {
//     isKickingTeam = true;
//     gameState.state = gameState.stateForOtherTeam();
//     timeWhenKickingTeamWasOverridden = theFrameInfo.time;
//   }
// 
//   // If we assume that the free kick is for us, but we got an illegal position penalty,
//   // the free kick is for the opponent. We also assume this if both teams got illegal position
//   // penalties.
//   if(gameState.isForOwnTeam() &&
//      *std::max_element(illegalPositionTimestampsOwnTeam.begin(), illegalPositionTimestampsOwnTeam.end()) > gameState.timeWhenStateStarted)
//   {
//     isKickingTeam = false;
//     gameState.state = gameState.stateForOtherTeam();
//     timeWhenKickingTeamWasOverridden = theFrameInfo.time;
//   }
// }

GameState::Phase GameStateProvider::convertGameControllerDataToPhase(const GameControllerData& gameControllerData)
{
  return (gameControllerData.gamePhase == GAME_PHASE_PENALTY_SHOOT_OUT)
             ? GameState::penaltyShootout
             : (gameControllerData.firstHalf ? GameState::firstHalf : GameState::secondHalf);
}

GameState::State GameStateProvider::convertGameControllerDataToState(const GameControllerData& gameControllerData)
{
  const bool isKickingTeamValid = (gameControllerData.kickingTeam != KICKING_TEAM_NONE);
  if(isKickingTeamValid)
    isKickingTeam = gameControllerData.isOwnTeamKick();

  if(gameControllerData.gamePhase == GAME_PHASE_TIMEOUT)
  {
    ASSERT(gameControllerData.state == STATE_INITIAL);
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::timeout;
  }
  else if(gameControllerData.gamePhase == GAME_PHASE_PENALTY_SHOOT_OUT)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    ASSERT(isKickingTeamValid);
    if (gameControllerData.state == STATE_INITIAL)
      return GameState::beforePenaltyShootout;
    else if (gameControllerData.state == STATE_SET)
      return isKickingTeam ? GameState::waitForOwnPenaltyShot : GameState::waitForOpponentPenaltyShot;
    else if (gameControllerData.state == STATE_PLAYING)
      return isKickingTeam ? GameState::ownPenaltyShot : GameState::opponentPenaltyShot;
    else if (gameControllerData.state == STATE_FINISHED)
      return isKickingTeam ? GameState::afterOwnPenaltyShot : GameState::afterOpponentPenaltyShot;
    else
      FAIL("Invalid/impossible game state.");
  }

  ASSERT((gameControllerData.gamePhase == GAME_PHASE_NORMAL) || (gameControllerData.gamePhase == GAME_PHASE_EXTRA_TIME));
  if(gameControllerData.state == STATE_INITIAL)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::beforeHalf;
  }
  else if(gameControllerData.state == STATE_STANDBY)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::standby;
  }
  else if(gameControllerData.state == STATE_READY)
  {
    if(gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
    {
      // Is the pushing free kick a new state?
      // if(!isKickingTeamValid && !(GameState::isPenaltyKick(lastGameControllerState) && GameState::isReady(lastGameControllerState)))
      //   updateKickingTeamForPushing(gameControllerData);
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::setupOwnPenaltyKick : GameState::setupOpponentPenaltyKick;
    }
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      return !isKickingTeamValid ? GameState::setupDropBall
             : isKickingTeam ? GameState::setupOwnKickOff : GameState::setupOpponentKickOff;
    }
  }
  else if(gameControllerData.state == STATE_SET)
  {
    if(gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
      return isKickingTeam ? GameState::waitForOwnPenaltyKick : GameState::waitForOpponentPenaltyKick;
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      return !isKickingTeamValid ? GameState::waitForDropBall
             : isKickingTeam ? GameState::waitForOwnKickOff : GameState::waitForOpponentKickOff;
    }
  }
  else if(gameControllerData.state == STATE_PLAYING)
  {
    if (gameControllerData.setPlay == SET_PLAY_PENALTY_KICK)
      return isKickingTeam ? GameState::ownPenaltyKick : GameState::opponentPenaltyKick;
    // else if (gameControllerData.setPlay == SET_PLAY_PUSHING_FREE_KICK)
    // {
    //   // Is the pushing free kick a new state?
    //   if (!isKickingTeamValid && !GameState::isPushingFreeKick(lastGameControllerState))
    //     updateKickingTeamForPushing(gameControllerData);
    //   return isKickingTeam ? GameState::ownPushingFreeKick : GameState::opponentPushingFreeKick;
    // }
    else if (gameControllerData.setPlay == SET_PLAY_DIRECT_FREE_KICK)
    {
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::ownDirectFreeKick : GameState::opponentDirectFreeKick;
    }
    else if (gameControllerData.setPlay == SET_PLAY_INDIRECT_FREE_KICK)
    {
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::ownIndirectFreeKick : GameState::opponentIndirectFreeKick;
    }
    else if (gameControllerData.setPlay == SET_PLAY_THROW_IN)
    {
      // if (!isKickingTeamValid && !GameState::isKickIn(lastGameControllerState))
      //   isKickingTeam = false;
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::ownThrowIn : GameState::opponentThrowIn;
    }
    else if (gameControllerData.setPlay == SET_PLAY_GOAL_KICK)
    {
      // if (!isKickingTeamValid)
      // {
      //   if (!gameStateOverridden && theTeamBallModel.isValid)
      //     isKickingTeam = theTeamBallModel.position.x() < 0;
      //   else if (!GameState::isGoalKick(lastGameControllerState))
      //     isKickingTeam = false;
      // }
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::ownGoalKick : GameState::opponentGoalKick;
    }
    else if (gameControllerData.setPlay == SET_PLAY_CORNER_KICK)
    {
      // if (!isKickingTeamValid)
      // {
      //   if (!gameStateOverridden && theTeamBallModel.isValid)
      //     isKickingTeam = theTeamBallModel.position.x() > 0;
      //   else if (!GameState::isCornerKick(lastGameControllerState))
      //     isKickingTeam = false;
      // }
      ASSERT(isKickingTeamValid);
      return isKickingTeam ? GameState::ownCornerKick : GameState::opponentCornerKick;
    }
    else
    {
      ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
      if (gameControllerData.secondaryTime > 0)
      {
        // The GameController doesn't explicitly have a state for this in the network messages,
        // but a secondaryTime != 0 can only happen during kick-off. This only happens when the
        // GameController's 15s delay after the whistle ends early due to an event that could
        // not occur during SET.
        return isKickingTeam ? GameState::ownKickOff : GameState::opponentKickOff;
      }
      else
        return GameState::playing;
    }
  }
  else if(gameControllerData.state == STATE_FINISHED)
  {
    ASSERT(gameControllerData.setPlay == SET_PLAY_NONE);
    return GameState::afterHalf;
  }
  else
    FAIL("Invalid/Impossible game state.");

  return GameState::playing; // default - only used if FAIL in release mode compiled out
}

GameState::PlayerState GameStateProvider::convertPenaltyToPlayerState(decltype(RoboCup::RobotInfo::penalty) penalty)
{
  switch(penalty)
  {
  case PENALTY_NONE: return GameState::active;

  case PENALTY_MANUAL: return GameState::penalizedManual;
  case PENALTY_ILLEGAL_POSITIONING: return GameState::penalizedIllegalPositioning;
  case PENALTY_MOTION_IN_SET: return GameState::penalizedMotionInSet;
  case PENALTY_MOTION_IN_STOP: return GameState::penalizedMotionInStop;
  case PENALTY_LOCAL_GAME_STUCK: return GameState::penalizedLocalGameStuck;
  case PENALTY_INCAPABLE_ROBOT: return GameState::penalizedIncapableRobot;
  case PENALTY_PICK_UP: return GameState::penalizedRequestForPickup;
  case PENALTY_BALL_HOLDING: return GameState::penalizedBallHolding;
  case PENALTY_LEAVING_THE_FIELD: return GameState::penalizedLeavingTheField;
  case PENALTY_PLAYING_WITH_ARMS_HANDS: return GameState::penalizedPlayingWithArmsHands;
  case PENALTY_PUSHING: return GameState::penalizedPushing;
  case PENALTY_CAUTIONED: return GameState::penalizedCautioned;
  case PENALTY_SENT_OFF: return GameState::penalizedSentOff;

  case PENALTY_SUBSTITUTE: return GameState::substitute;
  default: FAIL("Unknown penalty.");
  }

  return GameState::active; // only in case of FAIL in release mode
}
