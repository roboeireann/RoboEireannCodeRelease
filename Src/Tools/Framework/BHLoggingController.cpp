/**
 * @file BHLoggingController.cpp
 *
 * This file implements the class @c BHLoggingController which the Logger
 * uses to determine when it should start/stop logging
 *
 * @author Arne Hasselbring
 * @author Rudi Villing
 */

#include "BHLoggingController.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/RobotSafetyState.h"
#include "Framework/Blackboard.h"
#include "Platform/Time.h"
#include "Streaming/InStreams.h"
#include "TextLogging/TextLogging.h"


DECL_TLOGGER(tlogger, "BHLoggingController", TextLogging::INFO);

BHLoggingController::BHLoggingController() :
  gameState(Blackboard::getInstance().alloc<GameState>("GameState")),
  robotSafetyState(Blackboard::getInstance().alloc<RobotSafetyState>("RobotSafetyState"))
{
  InMapFile stream("teamList.cfg");
  if(stream.exists())
  {
    TeamList teamList;
    stream >> teamList;
    for(const Team& team : teamList.teams)
      teams[team.number] = team.name;
  }

  InMapFile paramsStream("BHLoggingController.cfg");
  if (paramsStream.exists())
  {
    paramsStream >> params;
  }
}

BHLoggingController::~BHLoggingController()
{
  Blackboard::getInstance().free("GameState");
  Blackboard::getInstance().free("RobotSafetyState");
}

bool BHLoggingController::shouldLog(bool wasLogging) const
{
  // We want to log in most states, but there are exceptions:
  // - The INITIAL state is excluded.
  // - The FINISHED state is excluded, except for the first 10s if we have been logging before.
  // return !(gameState.isInitial(/* orStandby: */ false)
  //          || (gameState.isFinished() && !(wasLogging && Time::getTimeSince(gameState.timeWhenStateStarted) < 10000)));
  // return !((gameState.isFinished() && !(wasLogging && Time::getTimeSince(gameState.timeWhenStateStarted) < 10000)));

  bool isSoftwareStartupNow = isSoftwareStartup;
  isSoftwareStartup = false;

  if (!wasLogging) // should we start logging
  {
    LoggingCondition reason = undefined; // default assumption

    // Start if ANY start condition is met
    const bool anyStartTriggered =
        std::any_of(params.startConditions.begin(), params.startConditions.end(),
                    [this, isSoftwareStartupNow, &reason](LoggingCondition cond) -> bool
                    {
                      switch (cond)
                      {
                      // usually you should set prepare as the second start logging condition if you need
                      // logging to start again after finishing since softwareStart only ever fires once
                      case softwareStart: reason = softwareStart; return isSoftwareStartupNow;
                      case safetyStateOperational:
                        reason = safetyStateOperational;
                        return robotSafetyState.state == RobotSafetyState::operational;
                      case safetyStateSafetyTripped:
                        reason = safetyStateSafetyTripped;
                        return robotSafetyState.state == RobotSafetyState::safetyTripped;
                      case playerStatePrepare:
                        reason = playerStatePrepare;
                        return gameState.playerState == GameState::prepare;
                      case playerStateActive:
                        reason = playerStateActive;
                        return gameState.playerState == GameState::active;
                      case stateInitial: reason = stateInitial; return gameState.isInitial(false);
                      case stateReady: reason = stateReady; return gameState.isReady();
                      case stateSet: reason = stateSet; return gameState.isSet();
                      case statePlaying: reason = statePlaying; return gameState.isPlaying();
                      case penaltyShootoutInitial:
                        reason = penaltyShootoutInitial;
                        return gameState.state == GameState::beforePenaltyShootout;
                      case penaltyShootoutShotSet:
                        reason = penaltyShootoutShotSet;
                        return (gameState.state == GameState::waitForOwnPenaltyShot) ||
                               (gameState.state == GameState::waitForOpponentPenaltyShot);
                      case penaltyShootoutShotPlaying:
                        reason = penaltyShootoutShotPlaying;
                        return (gameState.state == GameState::ownPenaltyShot) ||
                               (gameState.state == GameState::opponentPenaltyShot);
                      case penaltyShootoutShotFinished:
                        reason = penaltyShootoutShotFinished;
                        return (gameState.state == GameState::afterOwnPenaltyShot) ||
                               (gameState.state == GameState::afterOpponentPenaltyShot);
                      default: // unstiff, finished, etc don't make sense for start conditions
                        reason = undefined;
                        return gameState.isReady() || gameState.isSet() || gameState.isPlaying();
                      }
                    });

    if (anyStartTriggered)
      TLOGI(tlogger, "shouldLog: start logging, reason={}", TypeRegistry::getEnumName(reason));

    return anyStartTriggered;
  }
  else // should we continue logging (true) or stop logging (false)?
  {
    const bool safetyStateLongEnough =
        robotSafetyState.timeWhenStateStarted
            ? Time::getTimeSince(robotSafetyState.timeWhenStateStarted) > params.stopConditionTimeout
            : false;
    const bool playerStateLongEnough =
        gameState.timeWhenPlayerStateStarted
            ? Time::getTimeSince(gameState.timeWhenPlayerStateStarted) >= params.stopConditionTimeout
            : false;
    const bool gameStateLongEnough =
        gameState.timeWhenStateStarted ? Time::getTimeSince(gameState.timeWhenStateStarted) >= params.stopConditionTimeout
                                       : false;

    LoggingCondition reason = undefined; // default assumption

    // Stop logging (return false) if ANY stop condition triggers
    const bool anyStopTriggered = std::any_of(
        params.stopConditions.begin(), params.stopConditions.end(),
        [this, playerStateLongEnough, gameStateLongEnough, safetyStateLongEnough, &reason](LoggingCondition cond) -> bool
        {
          switch (cond)
          {
          case safetyStateOperational:
            reason = softwareStart;
            return (robotSafetyState.state == RobotSafetyState::operational) && safetyStateLongEnough;
          case safetyStateSafetyTripped:
            reason = safetyStateSafetyTripped;
            return (robotSafetyState.state == RobotSafetyState::safetyTripped) && safetyStateLongEnough;
          case playerStateUnstiff:
            reason = playerStateUnstiff;
            return (gameState.playerState == GameState::unstiff && playerStateLongEnough);
          case playerStatePrepare:
            reason = playerStatePrepare;
            return (gameState.playerState == GameState::prepare && playerStateLongEnough);
          case stateReady: reason = stateReady; return (gameState.isReady() && gameStateLongEnough);
          case stateSet: reason = stateSet; return (gameState.isSet() && gameStateLongEnough);
          case statePlaying: reason = statePlaying; return (gameState.isPlaying() && gameStateLongEnough);
          case stateFinished: reason = stateFinished; return (gameState.isFinished() && gameStateLongEnough);
          case penaltyShootoutShotSet:
            reason = penaltyShootoutShotSet;
            return ((gameState.state == GameState::waitForOwnPenaltyShot) ||
                    (gameState.state == GameState::waitForOpponentPenaltyShot)) &&
                   gameStateLongEnough;
          case penaltyShootoutShotPlaying:
            reason = penaltyShootoutShotPlaying;
            return ((gameState.state == GameState::ownPenaltyShot) ||
                    (gameState.state == GameState::opponentPenaltyShot)) &&
                   gameStateLongEnough;
          case penaltyShootoutShotFinished:
            reason = penaltyShootoutShotFinished;
            return ((gameState.state == GameState::afterOwnPenaltyShot) ||
                    (gameState.state == GameState::afterOpponentPenaltyShot)) &&
                   gameStateLongEnough;
          default:       // most states don't make sense to stop logging
            return false; // not a stop condition, i.e. continue logging
          }
        });

    if (anyStopTriggered)
      TLOGI(tlogger, "shouldLog: stop logging, reason={} present for long enough", TypeRegistry::getEnumName(reason));

    return anyStopTriggered ? false : true; // should we continue logging?
  }
}

std::string BHLoggingController::getDescription() const
{
  if(gameState.gameControllerActive)
  {
    auto team = teams.find(static_cast<std::uint8_t>(gameState.opponentTeam.number));
    if(team != teams.end())
      return team->second + "_"
             + (gameState.phase == GameState::penaltyShootout ? "shootout"
                : gameState.phase == GameState::firstHalf ? "half1" : "half2");
  }
  return "NoGC";
}
