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

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/GotoSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ArmSkills.h"

namespace CoroBehaviour
{

  /**
   * This task handles most of the RoboCup robot states.
   * However the Unstiff and Penalized states are handled at a higher level
   * by the CoroBehaviourControl2022 class
   */
  CRBEHAVIOUR(SoccerGameControlTask)
  {
    CRBEHAVIOUR_INIT(SoccerGameControlTask) {}

    // void registerPossibleSubtasks() override
    // {
    //   REGISTER_BASIC_CORO(SimpleSetStateTask);
    //   REGISTER_BASIC_CORO(WalkAroundTask);
    //   REGISTER_BASIC_CORO(PlaceholderTask);
    // }

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
            commonSkills.activityStatus(BehaviorStatus::fallen);
            commonSkills.lookForward();
            motionSkills.getUp();
            CR_YIELD();
          }
        }
        else if (theRobotInfo.mode == RobotInfo::calibration)
        {
          annotation("Autonomous calibration mode.");
          CR_CHECKPOINT(calibrating);          
          while (theRobotInfo.mode == RobotInfo::calibration)
          {
            CALL_CARD(CalibrationCard); // use the BH2021 calibration routine for now
            CR_YIELD();
          }
        }
        else
        {
          if (theGameInfo.state == STATE_INITIAL)
          {
            CR_CHECKPOINT(initial);
            commonSkills.activityStatus(BehaviorStatus::initial);
            commonSkills.standLookForward(true);
          }
          else if (theGameInfo.state == STATE_FINISHED)
          {
            CR_CHECKPOINT(finished);
            commonSkills.activityStatus(BehaviorStatus::finished);
            commonSkills.standLookForward(true);
          }
          else if (theGameInfo.state == STATE_SET)
          {
            CR_CHECKPOINT(set_state);
            commonSkills.activityStatus(BehaviorStatus::set);
            CALL_BASIC_CORO(params.setStateBehaviour); // the configured set state behaviour
          }
          else
          {
            commonSkills.activityStatus(BehaviorStatus::generalPlay);

            // we are not allowed move in the initial, finished, or set states, so
            // we only handle potential return from penalty (which may involve moving) here

            // Check if we need to any arm avoidance for obstacles
            armContactAnyArmTask();
            if (!theRobotInfo.isGoalkeeper())
              armObstacleAvoidanceAnyArmTask();

            if ((theExtendedGameInfo.timeSinceLastPenaltyEnded == 0) || (returnFromPenalizedTask.isYielded()))
            {
              CR_CHECKPOINT(return_from_penalized);
              returnFromPenalizedTask();
            }
            else if (theGameInfo.state == STATE_READY)
            {
              CR_CHECKPOINT(ready_state);
              CALL_BASIC_CORO(params.readyStateBehaviour); // the configured ready state behaviour
            }
            else if (theGameInfo.state == STATE_PLAYING)
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
      (std::string) setStateBehaviour,
      (std::string) readyStateBehaviour,
      (std::string) playingStateBehaviour,
      (std::string) returnFromPenalizedBehaviour,
    });

    READS(RobotInfo);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    READS(FallDownState);
    READS(MotionInfo);

    CommonSkills commonSkills  {env};
    MotionSkills motionSkills  {env};

    //For arm motion tasks
    ArmContactAnyArmTask armContactAnyArmTask {env};
    ArmObstacleAvoidanceAnyArmTask armObstacleAvoidanceAnyArmTask {env};

    DECLARE_CALLS_BASIC_CORO(returnFromPenalizedTask, params.returnFromPenalizedBehaviour);

    bool needsGetUp()
    {
      bool initial = (theGameInfo.state == STATE_INITIAL);
      bool finished = (theGameInfo.state == STATE_FINISHED);
      (void)finished; // force use
      
      bool fallen = (theFallDownState.state == FallDownState::fallen);
      bool accidentalSquatting = (theFallDownState.state == FallDownState::squatting &&
                                  // goalie sit down is not accidental so exclude it
                                  !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDownKeeper) &&
                                  // specifically requesting a sit down is not accidental so exclude it
                                  !theMotionInfo.isKeyframeMotion(KeyframeMotionRequest::sitDown));

      // return (fallen || accidentalSquatting) && !(initial || finished);
      return (fallen || accidentalSquatting) && !initial; // we do allow a get up in finished state
    }
  };

} // CoroBehaviour