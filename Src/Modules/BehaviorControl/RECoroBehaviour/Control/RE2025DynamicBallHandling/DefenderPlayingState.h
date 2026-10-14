/**
 * @file DefenderPlayingState.h
 *
 * This task implements the top level playing state behaviour for all 
 * defender robots in the dynamic ball handling challenge.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "InterceptSkills.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/GotoSkills.h"

#include "Math/Pose2f.h"


namespace RE
{
namespace v2025 
{ 
namespace DBHC
{
  
  // ==========================================================================

  CRBEHAVIOUR(DefenderGotoPoseAndStandTask)
  {
    CRBEHAVIOUR_INIT(DefenderGotoPoseAndStandTask) {}

    void operator()(const Pose2f& target, float walkSpeed = 1.f, bool lookAtBall = true)
    {
      CRBEHAVIOUR_LOOP()
      {
        // look at the ball if we saw it recently
        if (lookAtBall)
          // headSkills.lookAtBall(); // TODO: do we need to force looking at own ball model only?
          headSkills.lookActive(/* withBall: */ true); // TODO: should we look active instead of just looking at the ball and should distance factor in
        else // look around for the ball
          lookLeftAndRightTask();

        walkToPoseAndStandTask(target, walkSpeed, /*fineTuneDurationMs*/ 0, /*minOutOfPositionDurationMs*/ 0);

        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    
    HeadSkills headSkills {env};

    LookLeftAndRightTask lookLeftAndRightTask {env};
    WalkToPoseAndStandTask walkToPoseAndStandTask {env};
  };

  // ==========================================================================

  CRBEHAVIOUR(DefenderInterceptTask)
  {
    CRBEHAVIOUR_INIT(DefenderInterceptTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.recordActivity(BehaviorStatus::reIntercept);
        interceptTask(InterceptTask::DEFENDER_INTERCEPTS);
        if (!interceptTask.isSuccess())
          CR_YIELD();
        else
          CR_EXIT_SUCCESS();
      }
    }

    bool needsIntercept()
    {
      return theFieldBall.ballWasSeen(params.ballSeenInterceptTimeoutMs) &&
          params.interceptYRange.isInside(std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y())) &&
          params.interceptTimeRangeSecs.isInside(theFieldInterceptBall.timeUntilIntersectsOwnYAxis) &&
          (theFieldBall.positionRelative.norm() < params.ballInterceptDistance);
    }

  private:
    DEFINES_PARAMS(DefenderInterceptTask,
    {,
      (unsigned)(500) ballSeenInterceptTimeoutMs, // we can only intercept the ball if seen very recently
      (Rangef)(80.f, 400.f) interceptYRange, ///< if the ball intersects inside foot distance, don't move, otherwise check up to max dive range
      (Rangef)(0.3f, 3.f) interceptTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
      (float)(2000.f) ballInterceptDistance, ///< don't try to intercept ball unless it is closer than this (mm)
    });

    READS(FieldBall);
    READS(FieldInterceptBall);

    CommonSkills commonSkills {env};
    InterceptTask interceptTask {env};
  };

  // ==========================================================================

  /**
   * search for the ball
   * First check the team ball location if it is valid
   * If not there or nothing there, do a rotation until facing opponent,
   * scan a few times, and repeat
   * 
   * We assume that the calling task will switch away from this behaviour
   * as soon as the field ball is seen again
   */
  CRBEHAVIOUR(DefenderSearchForBallTask)
  {
    CRBEHAVIOUR_INIT(DefenderSearchForBallTask) {}

    void operator()(const Vector2f& fallbackPointOnField)
    {
      // do this no matter where we resume in the coro
      // commonSkills.recordActivity(BehaviorStatus::reSearchForBall);

      CRBEHAVIOUR_LOOP()
      {

        CR_CHECKPOINT(look_at_team_ball); // ...if it is valid

        // FIXME: Our old behaviour use to look at the team ball position for
        // a period after it was last valid. BH2025 does not directly support that.
        // For now we'll just check if the teamBall is valid, but that might break
        // or degrade our old behaviour - needs testing.
        // while (theFieldBall.timeSinceTeamBallWasValid < params.teamBallSeenTimeoutMs)
        while (theFieldBall.teamBallIsValid)
        {
          // look at the team ball, rotating if needed
          if (std::abs(theFieldBall.teamEndPositionRelative.angle()) < params.headTurnAngle)
          {
            headSkills.lookAtGlobalBall();
            commonSkills.stand();
          }
          else
          {
            headSkills.lookActive();
            walkToPoseAutoAvoidanceTask(Pose2f(theFieldBall.teamEndPositionRelative.angle(), 0.f, 0.f),
                                        Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed));
          }

          if (theFieldBall.ballWasSeen())
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();
        }

        chooseTurnDirection();
        // FIXME: see commend above
        // while (!(theFieldBall.timeSinceTeamBallWasValid < params.teamBallSeenTimeoutMs))
        // FIXME: the following bit of behaviour also looks a bit suspicious
        while (theFieldBall.teamBallIsValid)
        {
          CR_CHECKPOINT(turn_and_search);
          while (!turnDirectionOdometryTask.isSuccess())
          {
            lookLeftAndRightTask(turnDirection);
            turnDirectionOdometryTask(360_deg * turnDirection);
            if (theFieldBall.ballWasSeen())
              CR_EXIT_SUCCESS();
            else
              CR_YIELD();
          }

          CR_CHECKPOINT(turn_to_fallback_point);
          while (!turnToPointOdometryTask.isSuccess())
          {
            lookLeftAndRightTask(turnDirection);
            turnToPointOdometryTask(theRobotPose.toRobotCoordinates(fallbackPointOnField));
            if (theFieldBall.ballWasSeen())
              CR_EXIT_SUCCESS();
            else
              CR_YIELD();
          }

          // we've done a turn, so face forward (opponent half) and scan 
          // left to right for a while before repeating
          CR_CHECKPOINT(stand_and_scan);
          while (getCheckpointDuration() < params.standAndScanMs)
          {
            lookLeftAndRightTask(turnDirection);
            commonSkills.stand();
            if (theFieldBall.ballWasSeen())
              CR_EXIT_SUCCESS();
            else
              CR_YIELD();
          }
        }
      }
    }

  private:
    DEFINES_PARAMS(DefenderSearchForBallTask,
    {,
      (int)(2000) teamBallSeenTimeoutMs, // has the team seen the ball recently
      (Angle)(50_deg) headTurnAngle, // don't turn the head more than this when looking for the ball
      (float)(1.f) walkSpeed,
      (unsigned)(8000) standAndScanMs,
    });

    READS(RobotPose);
    READS(FieldBall);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    TurnDirectionOdometryTask turnDirectionOdometryTask {env};
    TurnToPointOdometryTask turnToPointOdometryTask {env};


    int turnDirection; // 1 or -1

    void chooseTurnDirection()
    {
      // Always turn in-field (towards the centre of the field) first
      // - theRobotPose is the robot relative to the field origin 
      //   and theRobotPose.inversePose is the field origin relative to the robot
      // - get the bearing to the field origin point (theRobotPose.inversePose.translation) 
      //   from the robot. (It will lie between -180 and 180 degrees)
      // - if it is between 0 and -180 the field origin is clockwise from the robot
      //   otherwise it is anticlockwise
      Angle originBearing = theRobotPose.inverse().translation.angle();

      turnDirection = (originBearing > 0) ? 1 : -1;      
    }   
  };



  // ==========================================================================
  
  /** 
   * walk to intercept any passing opportunities in our own half by simply
   * tracking the ball
   */
  CRBEHAVIOUR(DefendingSimpleDefenderPlayingTask)
  {
    CRBEHAVIOUR_INIT(DefendingSimpleDefenderPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (defenderInterceptTask.needsIntercept())
          CR_AWAIT(defenderInterceptTask.isSuccess(), defenderInterceptTask());
        else
        {
          if (theFieldBall.ballWasSeen(params.ballSeenActiveTimeoutMs))
          {
            // commonSkills.recordActivity(BehaviorStatus::reGotoPose);
            if (theFieldBall.endPositionOnField.x() < 100.f) // close to defense side of the field
                // CALL_SKILL(GoToBallAndDribble)(0.f); // it doesn't matter which direction we dribble in (this sets head and walk)
              gotoBallAndDribbleTask(0.f);
            else
              defenderGotoPoseAndStandTask(getBallInOpponentHalfPose(), /*walkSpeed*/ 1.f, /*lookAtBall*/ true);
          }
          else // didn't see the ball
          {
            if (getCoroDuration() < params.walkForwardMs)
            {
              // commonSkills.recordActivity(BehaviorStatus::reGotoPose);
              defenderGotoPoseAndStandTask(theRobotPose.toRobotCoordinates(Pose2f(0.f, -1000.f, 0.f)),
                                           /*walkSpeed*/ 1.f, /*lookAtBall*/ false);
            }
            else // still don't see the ball so search
              defenderSearchForBallTask(params.searchFallBackPointOnField);
          }  

          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(DefendingSimpleDefenderPlayingTask,
    {,
      (unsigned)(15000) walkForwardMs,
      (float)(1.f) walkSpeed,
      (unsigned)(5000) ballSeenActiveTimeoutMs, // TODO: should it be 7000?
      (Vector2f)({2000.f, 0.f}) searchFallBackPointOnField, // where the robot should look when not turning in search
    });

    READS(RobotPose);
    READS(FieldBall);
    READS(FieldDimensions);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    DefenderInterceptTask defenderInterceptTask {env};
    DefenderGotoPoseAndStandTask defenderGotoPoseAndStandTask {env};
    DefenderSearchForBallTask defenderSearchForBallTask {env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};

    // get a pose near the ball but don't bother to kick it
    Pose2f getBallInOpponentHalfPose()
    {
      // 300mm inside own half in line with the ball and oriented towards it
      Pose2f poseOnField(180_deg, -300.f, theFieldBall.endPositionOnField.y());
      return theRobotPose.toRobotCoordinates(poseOnField);
    }
  };


  // ==========================================================================
  
  /** 
   * walk to limit/cut off the attacker's passing opportunities
   */
  CRBEHAVIOUR(DefendingSimpleForwardPlayingTask)
  {
    CRBEHAVIOUR_INIT(DefendingSimpleForwardPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (defenderInterceptTask.needsIntercept())
          CR_AWAIT(defenderInterceptTask.isSuccess(), defenderInterceptTask());
        else
        {

          if (theFieldBall.ballWasSeen(params.ballSeenActiveTimeoutMs))
          {
            // commonSkills.recordActivity(BehaviorStatus::reGotoPose);
            if (theFieldBall.endPositionOnField.x() > 0)
              // CALL_SKILL(GoToBallAndDribble)(0.f); // it doesn't matter which direction we dribble in (this sets head and walk)
              gotoBallAndDribbleTask(0.f);
            else
              defenderGotoPoseAndStandTask(getBallInOwnHalfPose(), /*walkSpeed*/ 1.f, /*lookAtBall*/ true);
          }
          else // didn't see the ball
          {
            if (getCoroDuration() < params.walkForwardMs)
            {
              // commonSkills.recordActivity(BehaviorStatus::reGotoPose);
              defenderGotoPoseAndStandTask(
                  theRobotPose.toRobotCoordinates(Pose2f(0.f, theFieldDimensions.xPosOpponentPenaltyMark, 0.f)),
                  /*walkSpeed*/ 1.f, /*lookAtBall*/ false);
            }
            else // still don't see the ball so search
              defenderSearchForBallTask(params.searchFallBackPointOnField);
          }  

          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(DefendingSimpleForwardPlayingTask,
    {,
      (unsigned)(8000) walkForwardMs,
      (float)(1.f) walkSpeed,
      (unsigned)(5000) ballSeenActiveTimeoutMs, // TODO: should it be 7000?
      (unsigned)(500) ballSeenInterceptTimeoutMs, // we can only intercept the ball if seen very recently
      (Rangef)(80.f, 400.f) interceptYRange, ///< if the ball intersects inside foot distance, don't move, otherwise check up to max dive range
      (Rangef)(0.3f, 3.f) interceptTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
      (float)(2000.f) ballInterceptDistance, ///< don't try to intercept ball unless it is closer than this (mm)
      (Vector2f)({-2000.f, 0.f}) searchFallBackPointOnField, // where the robot should look when not turning in search
    });

    READS(RobotPose);
    READS(FieldBall);
    READS(FieldDimensions);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    DefenderGotoPoseAndStandTask defenderGotoPoseAndStandTask {env};
    DefenderInterceptTask defenderInterceptTask {env};
    DefenderSearchForBallTask defenderSearchForBallTask{env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};

    // get a pose near the ball but don't bother to kick it
    Pose2f getBallInOwnHalfPose()
    {
      // 300mm inside opposition half in line with the ball and oriented towards it
      Pose2f poseOnField(180_deg, 300.f, theFieldBall.endPositionOnField.y());
      return theRobotPose.toRobotCoordinates(poseOnField);
    }
  };


  // ==========================================================================
  
  CRBEHAVIOUR(DefendingGoaliePlayingTask)
  {
    CRBEHAVIOUR_INIT(DefendingGoaliePlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (needsIntercept())
        {
          while (!interceptTask.isSuccess())
          {
            // commonSkills.recordActivity(BehaviorStatus::reIntercept);
            interceptTask(GOALIE_INTERCEPTS);
            CR_YIELD();
          }
        }
        else
        {
          // commonSkills.recordActivity(BehaviorStatus::reGotoPose);

          // look at the ball if we saw it recently
          if (theFieldBall.ballWasSeen(params.ballSeenActiveTimeoutMs)) // FIXME: we should consider if team ball seen here also
            headSkills.lookAtBall(); // TODO: do we need to force looking at own ball model only?
            // headSkills.lookActive(/* withBall: */ true); // TODO: should we look active instead of just looking at the ball and should distance factor in
          else // look around for the ball
            lookLeftAndRightTask(); // TODO: is this a good enough search scan?

          walkToPoseAndStandTask(getGoaliePose(), /*walkSpeed*/ 1.f, /*fineTuneDurationMs*/ 0, /*minOutOfPositionDurationMs*/ 0);
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(DefendingGoaliePlayingTask,
    {,
      (unsigned)(5000) ballSeenActiveTimeoutMs, // TODO: should it be 7000?
      (unsigned)(500) ballSeenInterceptTimeoutMs, // we can only intercept the ball if seen very recently
      (Rangef)(80.f, 400.f) interceptYRange, ///< if the ball intersects inside foot distance, don't move, otherwise check up to max dive range
      (Rangef)(0.3f, 3.f) interceptTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
      (float)(2000.f) ballInterceptDistance, ///< don't try to intercept ball unless it is closer than this (mm)
    });

    static constexpr unsigned GOALIE_INTERCEPTS = bit(Interception::walk); // no dives allowed

    READS(RobotPose);
    READS(FieldBall);
    READS(FieldInterceptBall);
    READS(FieldDimensions);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    WalkToPoseAndStandTask walkToPoseAndStandTask {env};
    InterceptTask interceptTask {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};

    Pose2f getGoaliePose()
    {
      float side = theFieldBall.endPositionOnField.y() > 0 ? 1.f : -1.f; // left or right
      // choose a y position within the goal mouth that does bang up against the goal post
      float yPos = std::min(theFieldDimensions.yPosLeftGoal - 300.f, std::abs(theFieldBall.endPositionOnField.y() / 2.f)) * side;

      return theRobotPose.toRobotCoordinates(Pose2f(0_deg, theFieldDimensions.xPosOwnGoalLine, yPos));
    }

    bool needsIntercept()
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenInterceptTimeoutMs))
        return false;
      
      return params.interceptYRange.isInside(std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y())) &&
          params.interceptTimeRangeSecs.isInside(theFieldInterceptBall.timeUntilIntersectsOwnYAxis) &&
          (theFieldBall.positionRelative.norm() < params.ballInterceptDistance);
    }
  };


  // ==========================================================================
  
  /**
   * This is the entry point task for the playing state. It decides which
   * specialized behaviour to run
   */
  CRBEHAVIOUR(DefenderPlayingStateTask)
  {
    CRBEHAVIOUR_INIT(DefenderPlayingStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theTeamBehaviorStatus.role.isGoalkeeper())
          defendingGoaliePlayingTask();
        else if (theGameState.playerNumber == 3)
          defendingForwardPlayingTask();
        else
          defendingDefenderPlayingTask();

        CR_YIELD();
      }
    }

  private:
    READS(TeamBehaviorStatus);
    READS(GameState);
    
    CommonSkills commonSkills {env};

    DefendingGoaliePlayingTask defendingGoaliePlayingTask {env};
    DefendingSimpleForwardPlayingTask defendingForwardPlayingTask {env};
    DefendingSimpleDefenderPlayingTask defendingDefenderPlayingTask {env};
  };

} // DBHC
} // namespace v2025
} // namespace RE