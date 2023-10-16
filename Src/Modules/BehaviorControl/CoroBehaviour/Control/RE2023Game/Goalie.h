/**
 * @file Goalie.h
 *
 * This file contains the main goalie behaviours.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "GotoBallSkills.h"
#include "GoalieSaveSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"



namespace CoroBehaviour
{
namespace RE2023
{

  // =====================================================================

  CRBEHAVIOUR(WalkToGoaliePoseTask)
  {
    CRBEHAVIOUR_INIT(WalkToGoaliePoseTask) {}

    void operator()(void)
    {
      updateGoaliePose(); // do this on every tick before resuming our place in the coro body

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walk_to_goalie_pose);
        do
        {
          walkToPoseAutoAvoidanceTask(goaliePose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                                        /* keepTargetRotation */ true,
                                                        params.distanceThreshold, params.angleThreshold);
          CR_YIELD();
        }
        while (!walkToPoseAutoAvoidanceTask.isSuccess());


        CR_CHECKPOINT(fine_tune_goalie_pose);
        while (getCheckpointDuration() < params.fineTuneDurationMs)
        {
          walkToPoseAutoAvoidanceTask(goaliePose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                                         /* keepTargetRotation */ false,
                                                         params.distanceThreshold, params.angleThreshold);
          CR_YIELD();                                            
        }

        CR_CHECKPOINT(stand_at_goalie_pose);
        while (!outOfPosition() || (getCheckpointDuration() < params.minOutOfPositionDurationMs))
        {
          motionSkills.stand();
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(WalkToGoaliePoseTask,
    {,
      (float)(1.f) walkSpeed,
      (float)(100.f) distanceThreshold, // mm
      (float)(10_deg) angleThreshold,
      (unsigned)(2000) fineTuneDurationMs,
      (unsigned)(2000) minOutOfPositionDurationMs,
      (float)(200.f) outOfPositionDistance, // mm
      (float)(20_deg) outOfPositionAngle,
    });

    READS(FieldDimensions);
    READS(RobotPose);

    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Pose2f goaliePose; // robot-relative goalie pose

    void updateGoaliePose()
    {
      // FIXME - hardcoded to centre of goal, facing straight out for now
      goaliePose = theRobotPose.toRobotCoordinates(Pose2f(0_deg, theFieldDimensions.xPosOwnGroundLine, 0.f));
    }

    bool outOfPosition()
    {
      return !commonSkills.isPoseClose(goaliePose, params.outOfPositionDistance, params.outOfPositionAngle);
    }
  };

  // =====================================================================

  CRBEHAVIOUR(GotoGoaliePoseTask)
  {
    CRBEHAVIOUR_INIT(GotoGoaliePoseTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // look at the ball if we saw it recently
        if (theFieldBall.ballWasSeen(params.ballSeenActiveTimeoutMs)) // FIXME: we should consider if team ball seen here also
          headSkills.lookAtBall(); // TODO: do we need to force looking at own ball model only?
          // headSkills.lookActive(/* withBall: */ true); // TODO: should we look active instead of just looking at the ball and should distance factor in
        else // look around for the ball
          lookLeftAndRightTask(); // TODO: is this a good enough search scan?

        walkToGoaliePoseTask();

        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoGoaliePoseTask,
    {,
      (unsigned)(5000) ballSeenActiveTimeoutMs, // TODO: should it be 7000?
    });

    READS(FieldBall);

    HeadSkills headSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};
    WalkToGoaliePoseTask walkToGoaliePoseTask {env};
  };

  // =====================================================================

  CRBEHAVIOUR(GoaliePlayingTask)
  {  
    CRBEHAVIOUR_INIT(GoaliePlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (needsSaveNow())
        {
          do
          {
            // commonSkills.activityStatus(BehaviorStatus::reGoalieSave);
            goalieSaveTask(params.enabledSaves);
            CR_YIELD();
          }
          while (!goalieSaveTask.isSuccess());
        }
        else if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK && commonSkills.isOurTeamKick()) 
        {
          // commonSkills.activityStatus(BehaviorStatus::reKickBestOption);
          gotoBallAndKickBestOptionTask(true);
          CR_YIELD();
        } 
        else if (needsClearanceKick())
        {
          // commonSkills.activityStatus(BehaviorStatus::reKickBestOption);
          gotoBallAndKickBestOptionTask();
          CR_YIELD();
        }
        else
        {
          // commonSkills.activityStatus(BehaviorStatus::reGotoPose);
          gotoGoaliePoseTask();
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(GoaliePlayingTask,
    {,
      (unsigned)(300) ballSeenSaveTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(80.f, 800.f) saveYRange, ///< if the ball intersects inside foot distance, don't move, otherwise check up to max dive range
      (Rangef)(0.5f, 3.f) saveTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
      (float)(2500.f) ballSaveDistance, ///< don't try to save ball unless it is closer than this
      (float)(1600.f) kickAwayXMax, ///< we will go to the ball and kick it away in these x distances from ground line
      (float)(1500.f) kickAwayYMax, ///< maximum y-value at which we kick away. Together With kickAwayXRange, this defines a box
      (unsigned)(3000) kickAwayBallSeenTimeoutMs, ///< we'll allow the ball to be obstructed briefly on the way to kicking it away
      (unsigned)(bit(Interception::genuflectStand) | bit(Interception::jumpLeft) | bit(Interception::jumpRight)) enabledSaves,
    });

    READS(FieldBall);
    READS(FieldDimensions);
    READS(GameInfo);

    CommonSkills commonSkills {env};

    GoalieSaveTask goalieSaveTask {env};
    GotoGoaliePoseTask gotoGoaliePoseTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};

    /**
     * does the ball need saving right now (E.g. using a dive or wide stance).#
     * Note: if the ball is coming too quickly we don't even bother
     */
    bool needsSaveNow()
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenSaveTimeoutMs) || !theFieldBall.isRollingTowardsOwnGoal)
        return false;
      
      // If we get this far the ball probably needs saving but we need to check if
      // we can feasibly save it. Note some additional checks will be made in
      // the goalie save skill (e.g. to check we're inside the goal box etc)

      return params.saveYRange.isInside(std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y())) &&
          params.saveTimeRangeSecs.isInside(theFieldBall.timeUntilIntersectsOwnYAxis) &&
          (theFieldBall.positionRelative.norm() < params.ballSaveDistance);
    }

    /**
     * is the ball situation such that our best bet is to walk out to the ball and
     * kick it away?
     */
    bool needsClearanceKick()
    {
      // 
      //if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK)
      //  return true;

      // TODO: the current implementation is pretty naive and just looks at the
      // ball position. It would be better to consider things like, where
      // are the opponent players, where are our defensive players, before
      // committing to go 1v1

      // NOTE: that we also assume that if the goalie is the closest robot
      // to the ball it will role switch to become the ball player, in which
      // case that behaviour will run (and kick the ball) rather than this one

      Vector2f ballPos = theFieldBall.endPositionOnField;

      float xThreshold = std::min(theFieldDimensions.xPosOwnGroundLine + params.kickAwayXMax,
                                  theFieldDimensions.xPosOwnGroundLine + 1700.f);

      // is the ball in the defensive zone of the field
      bool inClearanceXRange = (ballPos.x() < xThreshold);

      // is the ball within the kickAway y range
      bool inClearanceYRange = (fabs(ballPos.y()) <= params.kickAwayYMax);

      return theFieldBall.ballWasSeen(params.kickAwayBallSeenTimeoutMs) && inClearanceXRange && inClearanceYRange;
    }
  };

} // RE2023
} // CoroBehaviour
