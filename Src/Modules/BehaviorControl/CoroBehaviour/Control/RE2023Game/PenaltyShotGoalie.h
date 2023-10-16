/**
 * @file PenaltyShotGoalie.h
 *
 * This file contains the penalty shot goalie behaviours.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 * 
 * For 2022 rules there are 2 kinds of penalty shots:
 * - in game penalty shot for a foul in the penalty area (SET_PLAY_PENALTY_KICK)
 * - penalty shoot out after a drawn game (GAME_PHASE_PENALTYSHOOT)
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

  // ======================================================================

  // the ready state should only be used for an in-game penalty shot

  CRBEHAVIOUR(PenaltyShotGoalieReadyTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotGoalieReadyTask) {}

    void operator()(void)
    {
      Pose2f goaliePose = theRobotPose.toRobotCoordinates(Pose2f(0, theFieldDimensions.xPosOwnGroundLine + 30, 0));

      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(walk_to_goalie_pose);
      while (!walkToPoseAutoAvoidanceTask.isSuccess())
      {
        // commonSkills.activityStatus(BehaviorStatus::reWalkToTacticPose);
        headSkills.lookActive();
        walkToPoseAutoAvoidanceTask(goaliePose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                    /* keepTargetRotation */ false, params.distanceThreshold, params.angleThreshold);
        CR_YIELD();
      }

      CR_CHECKPOINT(fine_tune_goalie_pose);
      while (!walkToPoseNoAvoidanceTask.isSuccess())
      {
        // commonSkills.activityStatus(BehaviorStatus::reWalkToTacticPose);
        headSkills.lookActive();
        walkToPoseNoAvoidanceTask(goaliePose, Pose2f(params.fineSpeed, params.fineSpeed, params.fineSpeed),
                                  /* keepTargetRotation */ true, params.fineDistanceThreshold,
                                  params.fineAngleThreshold);
        CR_YIELD();
      }

      CR_CHECKPOINT(stand_at_goalie_pose);
      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::reStandAtTacticPose);
        headSkills.lookActive();
        motionSkills.stand();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(WalkToGoaliePoseTask,
    {,
      (float)(1.f) walkSpeed,
      (float)(0.7f) fineSpeed,
      (float)(100.f) distanceThreshold, // mm
      (float)(10_deg) angleThreshold,
      (float)(20.f) fineDistanceThreshold, // mm
      (float)(2_deg) fineAngleThreshold,
    });

    READS(RobotPose);
    READS(FieldDimensions);

    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    WalkToPoseNoAvoidanceTask walkToPoseNoAvoidanceTask {env};
  };

  // ======================================================================

  CRBEHAVIOUR(PenaltyShotGoalieSetStateTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotGoalieSetStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        motionSkills.stand();
          // headSkills.lookAtBall(/* mirrored */ false, /* forceOwnEstimate */ true);
        headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
            Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0),
            theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(BallSpecification);
    READS(FieldDimensions);

    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};
  };


  // =====================================================================

  CRBEHAVIOUR(PenaltyShotGoaliePlayingTask)
  {  
    CRBEHAVIOUR_INIT(PenaltyShotGoaliePlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // get into goalie sitting position

      CR_CHECKPOINT(goalie_sit);
      actionDone = false;
      while (!actionDone)
      {
        // commonSkills.activityStatus(BehaviorStatus::reGoalieWaiting);
        headSkills.lookAtBall(/* mirrored */ false, /* forceOwnEstimate */ true);
        actionDone = motionSkills.keyframeMotion(KeyframeMotionRequest::sitDownKeeper);
        CR_YIELD();
      }

      CR_CHECKPOINT(ready_to_save);
      while (!needsSave())
      {
        // commonSkills.activityStatus(BehaviorStatus::reGoalieWaiting);
        headSkills.lookAtBall(/* mirrored */ false, /* forceOwnEstimate */ true);
        actionDone = motionSkills.keyframeMotion(KeyframeMotionRequest::sitDownKeeper);
        CR_YIELD();
      }

      // if we get this far it is because needsSave was true
      CR_CHECKPOINT(goalie_save);
      while (!goalieSaveTask.isSuccess())
      {
        // commonSkills.activityStatus(BehaviorStatus::reGoalieSave);
        goalieSaveTask(params.enabledSaves, /* isSitting */ true, params.getUpAfterSaveDelayMs);
        CR_YIELD();
      }

      CR_CHECKPOINT(stand_after_save);
      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::reGoalieWaiting);
        motionSkills.stand();
        headSkills.lookActive();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(PenaltyShotGoaliePlayingTask,
    {,
      (unsigned)(300) ballSeenSaveTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(80.f, 800.f) saveYRange, ///< if the ball intersects inside foot distance, don't move, otherwise check up to max dive range
      (Rangef)(0.1f, 3.5f) saveTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
      (unsigned)(bit(Interception::genuflectFromSitting) | bit(Interception::jumpLeft) | bit(Interception::jumpRight)) enabledSaves,
      (unsigned)(6000) getUpAfterSaveDelayMs,
    });

    READS(FieldBall);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    GoalieSaveTask goalieSaveTask {env};

    bool actionDone;

    /**
     * does the ball need saving right now (E.g. using a dive or wide stance).#
     * Note: if the ball is coming too quickly we don't even bother
     */
    bool needsSave()
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenSaveTimeoutMs) || !theFieldBall.isRollingTowardsOwnGoal)
        return false;
      
      // If we get this far the ball probably needs saving but we need to check if
      // we can feasibly save it. Note some additional checks will be made in
      // the goalie save skill (e.g. to check we're inside the goal box etc)

      return params.saveYRange.isInside(std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y())) &&
          params.saveTimeRangeSecs.isInside(theFieldBall.timeUntilIntersectsOwnYAxis);
    }
  };

} // RE2023
} // CoroBehaviour2023