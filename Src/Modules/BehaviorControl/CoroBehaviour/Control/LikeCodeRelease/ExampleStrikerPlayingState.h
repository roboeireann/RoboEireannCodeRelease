/**
 * @file ExampleStrikerPlayingState.h
 *
 * This file implements a basic kickoff behaviour.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Tools/Math/BHMath.h"
#include "Tools/Math/Pose2f.h"

namespace CoroBehaviour
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
      commonSkills.recordActivity(BehaviorStatus::codeReleaseKickAtGoal);

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
      return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f)).angle();
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
        if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
        {
          if (theGameInfo.isOurKick())
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
        else if (theGameInfo.setPlay != SET_PLAY_NONE) // any other set play?
        {
          if (theGameInfo.isOurKick())
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
    READS(GameInfo);
    
    CommonSkills commonSkills  {env};
    ExampleStrikerNormalPlayTask normalPlayTask  {env};
  };


} // CoroBehaviour2022