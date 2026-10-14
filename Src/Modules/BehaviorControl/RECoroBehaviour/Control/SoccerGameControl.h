/**
 * @file GameControlTask.h
 *
 * Originally developed for RoboCup 2019 - Sydney...
 * This task manages the high level RoboCup game state transitions and
 * delegates to the appropriate sub tasks
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/GotoSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/ArmSkills.h"

#include "Framework/Settings.h"
#include "Streaming/Global.h"
#include "TextLogging/TextLogging.h"

namespace RE
{

  /**
   * This task handles most of the RoboCup robot states.
   * However the Unstiff and Penalized states are handled at a higher level
   * by the CoroBehaviourModuleYEAR
   */
  CRBEHAVIOUR(SoccerGameControlTask)
  {
    CRBEHAVIOUR_INIT(SoccerGameControlTask) 
    {
      TLOGI(tlogger, "standbyStateBehaviour: {}", params.standbyStateBehaviour);
      TLOGI(tlogger, "readyStateBehaviour: {}", params.readyStateBehaviour);
      TLOGI(tlogger, "setStateBehaviour: {}", params.setStateBehaviour);
      TLOGI(tlogger, "playingStateBehaviour: {}", params.playingStateBehaviour);
      TLOGI(tlogger, "returnFromPenalizedBehaviour: {}", params.returnFromPenalizedBehaviour);
    }

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (needsGetUp())
        {
          annotation("Getting up.");

          CR_CHECKPOINT(getting_up);
          while (theFallDownState.state != FallDownState::upright)
          {
            // commonSkills.recordActivity(BehaviorStatus::fallen);
            commonSkills.lookForward();
            motionSkills.stand();
            // motionSkills.getUp();
            CR_YIELD();
          }
        }
        else if (theGameState.playerState == GameState::calibration)
        {
          annotation("Autonomous calibration mode.");
          CR_CHECKPOINT(calibrating);          
          while (theGameState.playerState == GameState::calibration)
          {
            // CALL_CARD(CalibrationCard); // use the BH2021 calibration routine for now
            // FIXME - integrate new Calibration procedure
            commonSkills.lookForward();
            motionSkills.stand();
            CR_YIELD();
          }
        }
        else
        {
          if (theGameState.isInitial())
          {
            CR_CHECKPOINT(initial);
            // commonSkills.recordActivity(BehaviorStatus::initial);
            commonSkills.standLookForward(true);
          }
          else if (theGameState.state == GameState::standby)
          {
            CR_CHECKPOINT(standby);
            // commonSkills.recordActivity(BehaviorStatus::initial);
            CALL_BASIC_CORO(params.standbyStateBehaviour); // the configured playing state behaviour
          }
          else if (theGameState.isFinished())
          {
            CR_CHECKPOINT(finished);
            // commonSkills.recordActivity(BehaviorStatus::finished);
            commonSkills.standLookForward(true);
          }
          else if (theGameState.isSet())
          {
            CR_CHECKPOINT(set_state);
            // commonSkills.recordActivity(BehaviorStatus::set);
            CALL_BASIC_CORO(params.setStateBehaviour); // the configured set state behaviour
          }
          else if (theGameState.isMotionStoppedState())
          {
            CR_CHECKPOINT(stopped_state);
            // commonSkills.recordActivity(BehaviorStatus::stopped);
            commonSkills.standLookForward(true);
          }
          else
          {
            // commonSkills.recordActivity(BehaviorStatus::generalPlay);

            // we are not allowed move in the initial, finished, or set states, so
            // we only handle potential return from penalty (which may involve moving) here

            // Check if we need to any arm avoidance for obstacles
            armContactAnyArmTask();
            if (!theGameState.isGoalkeeper())
              armObstacleAvoidanceAnyArmTask();

            if (theExtendedGameState.returnFromGameControllerPenalty || theExtendedGameState.returnFromManualPenalty ||
                (returnFromPenalizedTask.isYielded()))
            {
              CR_CHECKPOINT(return_from_penalized);
              if (theExtendedGameState.returnFromGameControllerPenalty || theExtendedGameState.returnFromManualPenalty)
                CRB_ANNOTATION_FMT("returnFromPenalized start ({})",
                                   theExtendedGameState.returnFromGameControllerPenalty ? "from GC penalty"
                                                                                        : "from manual penalty");
              returnFromPenalizedTask();
            }
            else if (theGameState.isReady())
            {
              CR_CHECKPOINT(ready_state);
              CALL_BASIC_CORO(params.readyStateBehaviour); // the configured ready state behaviour
            }
            else if (theGameState.isPlaying())
            {
              CR_CHECKPOINT(playing_state);
              CALL_BASIC_CORO(params.playingStateBehaviour); // the configured playing state behaviour
            }
            else
              FAIL("unknown game state");
          }
          CR_YIELD();
        }
      }
    }

  private:
    LOADS_PARAMS(SoccerGameControlTask, 
    {,
      (std::string) standbyStateBehaviour,
      (std::string) readyStateBehaviour,
      (std::string) setStateBehaviour,
      (std::string) playingStateBehaviour,
      (std::string) returnFromPenalizedBehaviour,
    });

    // READS(REGameInfo);
    READS(GameState);
    READS(ExtendedGameState);
    READS(FallDownState);
    READS(MotionInfo);
    READS(FieldDimensions);
    READS(RobotPose);

    CommonSkills commonSkills  {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills  {env};

    //For arm motion tasks
    ArmContactAnyArmTask armContactAnyArmTask {env};
    ArmObstacleAvoidanceAnyArmTask armObstacleAvoidanceAnyArmTask {env};

    DECL_TLOGGER_STATIC_VAR(tlogger, "SoccerGameControlTask", TextLogging::INFO);

    DECLARE_CALLS_BASIC_CORO(returnFromPenalizedTask, params.returnFromPenalizedBehaviour);

    bool needsGetUp()
    {
      bool stopped = theGameState.isMotionStoppedState();
      bool set = theGameState.isSet();
      bool fallen = (theFallDownState.state == FallDownState::fallen);
      bool accidentalSquatting = (theFallDownState.state == FallDownState::squatting &&
                                  // goalie sit down is not accidental so exclude it
                                  !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDownKeeper) &&
                                  // specifically requesting a sit down is not accidental so exclude it
                                  !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDown));

      bool shouldSuppress = (stopped || set);

      return (fallen || accidentalSquatting) && !shouldSuppress; // we do allow a get up in finished state
    }
  };

} // namespace RE