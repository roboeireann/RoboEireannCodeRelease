/**
 * @file ExampleStrikerPlayingState.h
 *
 * This file implements a basic kickoff behaviour.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BallSkills.h"

#include "Tools/Math/BHMath.h"
#include "Math/Pose2f.h"

namespace RE
{

  // ============================================================================
  /**
   * Implement the normal playing behaviour for the simple striker.
   * Basically this striker just tries continuously to score a goal.
   * It does not take account of obstacles.
   */
  CRBEHAVIOUR(ExampleStrikerNormalPlayTask)
  {
    CRBEHAVIOUR_INIT(ExampleStrikerNormalPlayTask) {}

    void operator()(void)
    {
      // commonSkills.recordActivity(BehaviorStatus::codeReleaseKickAtGoal);

      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(initialWait);
      while (getCoroDuration() < params.initialWaitMs)
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }

      while (true)
      {
        CR_CHECKPOINT(gotoBallAndKick);
        while (theFieldBall.ballWasSeen(params.ballNotSeenTimeoutMs))
        {
          gotoBallAndKickTask(calcAngleToGoal(), KickInfo::walkForwardsLeft);
          CR_YIELD();
        }

        // if we get here we need to search for the ball.
        // For now, this is just a turn on the spot implementation
        CR_CHECKPOINT(searchForBall);
        while (!theFieldBall.ballWasSeen())
        {
          commonSkills.lookForward();
          motionSkills.walkAtRelativeSpeed(Pose2f(params.walkSpeed, 0.f, 0.f));
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(ExampleStrikerNormalPlayTask,
    {,
      (float)(0.8f) walkSpeed,
      (CoroTime)(1000) initialWaitMs,
      (CoroTime)(7000) ballNotSeenTimeoutMs,
    });

    READS(FieldBall);
    READS(FieldDimensions);
    READS(RobotPose);

    CommonSkills commonSkills  {env};
    MotionSkills motionSkills  {env};
    GotoBallAndKickTask gotoBallAndKickTask  {env};

    Angle calcAngleToGoal() const
    {
      return theRobotPose.toRobotCoordinates(Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f)).angle();
    }
  };

  // ============================================================================
  /**
   * Implement the playing state behaviour for the simple striker.
   * We don't implement any set plays here.
   */
  CRBEHAVIOUR(ExampleStrikerPlayingStateTask)
  {
    CRBEHAVIOUR_INIT(ExampleStrikerPlayingStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theGameState.isPenaltyKick())
        {
          if (theGameState.isForOwnTeam())
          {
            CR_CHECKPOINT(our_penalty_kick);
            commonSkills.standLookForward(); // TODO: should be ownPenaltyKick behaviour
          }
          else
          {
            CR_CHECKPOINT(opponent_penalty_kick);
            commonSkills.standLookForward(); // TODO: should be opponentPenaltyKick behaviour
          }
        }
        else if (theGameState.isFreeKick()) // any other set play?
        {
          if (theGameState.isForOwnTeam())
          {
            CR_CHECKPOINT(our_set_play);
            commonSkills.standLookForward(); // TODO: should be ownFreeKick behaviour
          }
          else
          {
            CR_CHECKPOINT(opponent_set_play);
            commonSkills.standLookForward(); // TODO: should be opponentFreeKick behaviour
          }
        }
        else // not a set play
        {
          CR_CHECKPOINT(normal_play);
          normalPlayTask();
        }
        
        // TODO: we don't implement the countdown skills for now - are they useful???

        CR_YIELD(); // ensure we yield on each loop
      }
    }

  private:
    READS(GameState);
    
    CommonSkills commonSkills  {env};
    ExampleStrikerNormalPlayTask normalPlayTask  {env};
  };


} // namespace RE