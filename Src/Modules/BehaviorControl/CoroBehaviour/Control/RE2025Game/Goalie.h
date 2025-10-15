/**
 * @file Goalie.h
 *
 * This file contains the main goalie behaviours.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 * @author Aidan Colgan
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2025/CoroBehaviour2025.h"

#include "GotoBallSkills.h"
#include "GoalieSaveSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Tools/FmtCommonTypes.h"


namespace CoroBehaviour
{
namespace RE2025
{

  // =====================================================================

  CRBEHAVIOUR(WalkToGoaliePoseTask)
  {
    CRBEHAVIOUR_INIT(WalkToGoaliePoseTask) {}

    void operator()(void)
    {
      //updateGoaliePose(); // do this on every tick before resuming our place in the coro body

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walk_to_goalie_pose);
        do
        {//New Task to look back
          updateGoaliePose();
          headSkills.lookAtBall();
          ACTGRAPH_FMT("goaliePose: {}", goaliePose);
          walkToPoseAutoAvoidanceTask(goaliePose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      /*keepTargetRotation->*/ true, params.distanceThreshold, params.angleThreshold);
          CR_YIELD();
        }
        while (!walkToPoseAutoAvoidanceTask.isSuccess());


        CR_CHECKPOINT(fine_tune_goalie_pose);
        while (getCheckpointDuration() < params.fineTuneDurationMs)
        {
          updateGoaliePose();
          // lookLeftAndRightTask();
          headSkills.lookActive(/* with ball*/ true);
          ACTGRAPH_FMT("goaliePose (mod): {}", Pose2f(forwardAngle, goaliePose.translation));
          walkToPoseAutoAvoidanceTask(Pose2f(forwardAngle, goaliePose.translation.x(), goaliePose.translation.y()),
                                      Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      /*keepTargetRotation->*/ true, params.distanceThreshold, params.angleThreshold);
          CR_YIELD();                                            
        }

        CR_CHECKPOINT(stand_at_goalie_pose);
        while (!outOfPosition() || (getCheckpointDuration() < params.minOutOfPositionDurationMs))
        {
          updateGoaliePose();
          ACTGRAPH_FMT("OutofPos {},Mintime{}", !outOfPosition(),
                       (getCheckpointDuration() < params.minOutOfPositionDurationMs));
          ACTGRAPH_FMT("Distance: {},threshold:{},True:{}", goaliePose.translation.squaredNorm(),
                       sqr(params.outOfPositionDistance),
                       (goaliePose.translation.squaredNorm() < sqr(params.outOfPositionDistance)));
          headSkills.lookActive(/* withBall */ true);
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
      (float)(100.f) outOfPositionDistance, // mm
      (float)(20_deg) outOfPositionAngle,
    });

    READS(FieldDimensions);
    READS(RobotPose);
    READS(BallModel);

    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};

    LookLeftAndRightTask lookLeftAndRightTask {env};
    WalkToPoseAutoAvoidanceLookAroundTask walkToPoseAutoAvoidanceLookAroundTask {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Pose2f goaliePose; // robot-relative goalie pose

    Angle forwardAngle = (theFieldDimensions.xPosOpponentGroundLine <= 0 )? float(3.14159): 0.f;

    // FIXME - remove pose modification for lookback from here - ths should be part of teh walk task only
    void updateGoaliePose()
    {
      // FIXME - hardcoded to centre of goal, facing straight out for now
      goaliePose = Pose2f(0_deg, theFieldDimensions.xPosOwnGroundLine, 0.f);
      //Allow for margin ahead of Goalline as we end up walking into Goal otherwise

      Vector2f tacticPosOnField=goaliePose.translation;
      //Building off getTacticPoseTask()
      if (tacticPosOnField.x() + 300.0 >= theRobotPose.translation.x())
      {
        ACTGRAPH_FMT("WalkForward");
        // If we need to move upfield
        Angle angleToBall = Angle(0.f);
        Pose2f tacticPoseOnField(angleToBall, tacticPosOnField);
        goaliePose = theRobotPose.toRobotCoordinates(tacticPoseOnField);
      }
      else
      { // We need to move downfield, move diag
        // Convert tactiPosOnField to Robo Relative
        ACTGRAPH_FMT("WalkDiag");
        tacticPosOnField = theRobotPose.toRobotCoordinates(tacticPosOnField);

        //!!Maybe add logic angleToBall Not Used when backing up!!

        // Angle angleToBall = ballTimedOutReposition ? Angle(0.f) :faceDiag(tacticPosOnField);
        Angle angleToBall = faceDiag(tacticPosOnField);
        Pose2f tacticPoseOnField(angleToBall, tacticPosOnField);
        // RETURN ROBOT RELATIVE CO-ORDS
        // return theRobotPose.toRobotCoordinates(tacticPoseOnField);
        goaliePose = tacticPoseOnField;
      }
    }

    // FIXME remove
    Angle faceDiag(Vector2f tacticPosOnField)
    {
      //29/04/24 -Need it in Robot Relative co ords so can adjust angle easier
      Angle leftOrRightAngle = faceDiagLeftOrRight();
      ACTGRAPH_FMT("leftOrRightAngle4 = {}", leftOrRightAngle.fmt());

      Angle angleForward = (theRobotPose.inversePose.translation-tacticPosOnField).angle()+leftOrRightAngle;
      ACTGRAPH_FMT("InversePosField = {}", theRobotPose.inversePose.translation);
      ACTGRAPH_FMT("TacticPosField = {}", tacticPosOnField);
      ACTGRAPH_FMT("Angle = {}, Angle-180 = {}",
                      FmtAngle((theRobotPose.inversePose.translation - tacticPosOnField).angle()),
                      FmtAngle((theRobotPose.inversePose.translation - tacticPosOnField).angle() - 180_deg));
      return(angleForward);
    }

    Angle faceDiagLeftOrRight()
    {
      //Ball Model estimate is pos relative to robot, need in field to get right side
      Vector2f ballPos= theRobotPose.toFieldCoordinates(theBallModel.estimate.position);
      addActivationGraphOutput(fmt::format("BallModelPos: {{{},{}}}",ballPos.x(),ballPos.y()));
      bool rotateRight= ballPos.y()<0;
      addActivationGraphOutput(fmt::format("rotateRight: {}",rotateRight));

      // If we prefer to turn in pitchways for now
      // addActivationGraphOutput(fmt::format("TheRobotPose: {{{},{}}}",theRobotPose.translation.x(),theRobotPose.translation.y()));
      // bool rotateRight= theRobotPose.translation.y()>0;
      // addActivationGraphOutput(fmt::format("rotateRight: {}",rotateRight));
      

      return(rotateRight ? -90_deg : 90_deg);
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
          CR_UNTIL(goalieSaveTask.isSuccess(), goalieSaveTask(params.enabledSaves));
        }
        else if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK && commonSkills.isOurTeamKick()) 
        {
          gotoBallAndKickBestOptionTask();
          CR_YIELD();
        } 
        else if (needsClearanceKick())
        {
          gotoBallAndKickBestOptionTask(/* clearance */ true);
          CR_YIELD();
        }
        else
        {
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
      // (float)(1600.f) kickAwayXMax, ///< we will go to the ball and kick it away in these x distances from ground line
      // (float)(1500.f) kickAwayYMax, ///< maximum y-value at which we kick away. Together With kickAwayXRange, this defines a box
      (unsigned)(3000) kickAwayBallSeenTimeoutMs, ///< we'll allow the ball to be obstructed briefly on the way to kicking it away
      (unsigned)(bit(Interception::genuflectStand) | bit(Interception::jumpLeft) | bit(Interception::jumpRight)) enabledSaves,
    });

    READS(FieldBall);
    READS(FieldDimensions);
    READS(GameInfo);
    READS(BehaviourParams);

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
          params.saveTimeRangeSecs.isInside(theFieldBall.secsUntilIntersectsOwnYAxis) &&
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

      if (!theFieldBall.ballWasSeen(params.kickAwayBallSeenTimeoutMs))
        return false;

      Vector2f ballPos = theFieldBall.endPositionOnField;

      return (ballPos.x() <= (theFieldDimensions.xPosOwnGroundLine + theBehaviourParams.goalieClearanceBox.x())) &&
             (fabs(ballPos.y()) <= theBehaviourParams.goalieClearanceBox.y());
    }
  };

} // RE2025
} // CoroBehaviour
