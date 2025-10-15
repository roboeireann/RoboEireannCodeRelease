/**
 * @file FastestWalk.h
 *
 * This file implements the RC2025 Fastest Walk Leaderboard Challenge with dribbling logic
 * based on goToBallAndDribble replacing walkToPoseTask for waypoint-based movement.
 *
 * @author Mariia Solianyk
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/SoundSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Representations/Modeling/ObstacleModel.h"

namespace CoroBehaviour
{
namespace RC2025
{
namespace Leaderboards
{
  CRBEHAVIOUR(BallControlTask)
  {
    CRBEHAVIOUR_INIT(BallControlTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      while (getCheckpointDuration() < params.initialLookAroundMs)
      {
        headSkills.lookActive();
        commonSkills.stand();
        CR_YIELD();
      }

      if (!obstaclesInFront())
      {
        CR_CHECKPOINT(walk_no_obstacles);
        do
        {
          targetPose = getFinalTargetPose();
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower);
          CR_YIELD();
        } while (!commonSkills.isPoseClose(targetPose));

        CR_CHECKPOINT(stand);
        CR_WHILE(true, commonSkills.standLookForward(true));
      }
      else
      {
        // Each waypoint defined manually as do-while loop

        do
        {
          CR_CHECKPOINT(left_of_obstacle1);
          targetPose = toFieldPose(0_deg, theFieldDimensions.xPosOwnPenaltyArea + params.robotRadius,
                                   theFieldDimensions.yPosLeftPenaltyArea + params.smallRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ 0.5f, true, false, false, Pose2f(0.5f, 0.5f, 0.5f), Rangea(0_deg, -45_deg));
          CR_YIELD();
        } while (!((theRobotPose.translation.y() < theFieldDimensions.yPosLeftPenaltyArea + params.smallRadius) && (theRobotPose.translation.x() > theFieldDimensions.xPosOwnPenaltyArea + 200.f)));

        do
        {
          CR_CHECKPOINT(left_of_obstacle1_continued);
          targetPose = toFieldPose(90_deg, theFieldDimensions.xPosOwnPenaltyArea + params.robotRadius,
                                   theFieldDimensions.yPosLeftPenaltyArea - params.smallRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower, false, false, false, Pose2f(0.7f, 0.7f, 0.7f));
          CR_YIELD();
        } while (!(theRobotPose.translation.y() < (theFieldDimensions.yPosLeftPenaltyArea - params.smallRadius)));

        do
        {
          CR_CHECKPOINT(right_of_obstacle2);
          targetPose = toFieldPose(90_deg,
                                   ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-170.f,
                                   theFieldDimensions.yPosLeftGoalArea + 500.f);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ 0.3f, false, false, false, Pose2f(1.f, 1.f, 1.f), Rangea(150_deg, 90_deg));
          CR_YIELD();
        } while (!((theRobotPose.translation.y() < (theFieldDimensions.yPosLeftGoalArea + 500.f)) && (theRobotPose.translation.x() < (theFieldDimensions.xPosOwnPenaltyMark - 700.f))));

        do
        {
          CR_CHECKPOINT(behind_obstacle2);
          targetPose = toFieldPose(90_deg,
                                   (theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2,
                                   theFieldDimensions.yPosLeftGoalArea - params.robotRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ 0.7f,true, false, false, Pose2f(0.6f, 0.6f, 0.6f), Rangea(0_deg, -45_deg));
          CR_YIELD();
        } while (!(theRobotPose.translation.y() < (theFieldDimensions.yPosLeftGoalArea - params.robotRadius)));

        do
        {
          CR_CHECKPOINT(left_of_obstacle3);
          targetPose = toFieldPose(90_deg, theFieldDimensions.xPosOwnPenaltyMark + params.smallRadius,
                                   theFieldDimensions.yPosCenterGoal + params.robotRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower,false, false, false,  Pose2f(0.7f, 0.7f, 0.7f), Rangea(70_deg, 0_deg));
          CR_YIELD();
        } while (!((theRobotPose.translation.y() < (theFieldDimensions.yPosCenterGoal + params.robotRadius)) && (theRobotPose.translation.x() > (theFieldDimensions.xPosOwnPenaltyMark + params.smallRadius))));

        do
        {
          CR_CHECKPOINT(left_of_obstacle3_continued);
          targetPose = toFieldPose(90_deg, theFieldDimensions.xPosOwnPenaltyMark + params.robotRadius,
                                   theFieldDimensions.yPosCenterGoal - params.smallRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower,false, false, false);
          CR_YIELD();
        } while (!((theRobotPose.translation.y() < (theFieldDimensions.yPosCenterGoal - params.robotRadius)) && (theRobotPose.translation.x() > (theFieldDimensions.xPosOwnPenaltyMark + params.robotRadius))));

        do
        {
          CR_CHECKPOINT(right_of_obstacle4);
          targetPose = toFieldPose(180_deg,
                                   ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-120.f,
                                   theFieldDimensions.yPosRightGoalArea + 450.f);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower,false, false, false, Pose2f(1.f, 1.f, 1.f), Rangea(150_deg, 90_deg));
          CR_YIELD();
        } while (!((theRobotPose.translation.y() < (theFieldDimensions.yPosRightGoalArea + 500.f)) && (theRobotPose.translation.x() < (theFieldDimensions.xPosOwnPenaltyMark - 450.f))));

        do
        {
          CR_CHECKPOINT(behind_obstacle4);
          targetPose = toFieldPose(0_deg,
                                   (theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2,
                                   theFieldDimensions.yPosRightGoalArea - params.smallRadius);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ 0.2f,false, false, false);
          CR_YIELD();
        } while (!(theRobotPose.translation.y() < (theFieldDimensions.yPosRightGoalArea - params.smallRadius)));

        do
        {
          CR_CHECKPOINT(final_target);
          targetPose = toFieldPose(90_deg, theFieldDimensions.xPosOwnGoalArea,
                                   theFieldDimensions.yPosRightSideline - params.distanceExtra);
          headSkills.lookActive();
          targetDirection = targetPose.translation.angle();
          gotoBallAndDribbleTask(targetDirection, /* ignoreObstacles */ false, /* kickPower */ params.kickPower,false, false, false);
          //goToBallAndDribble(targetPose.rotation, true);
          CR_YIELD();
        } while (!commonSkills.isPoseClose(targetPose));

        CR_CHECKPOINT(stand);
        CR_WHILE(true, commonSkills.standLookForward(true));
      }
    }

  private:
    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};

    Pose2f targetPose;
    Angle targetDirection;

    DEFINES_PARAMS(BallControlTask,
      {,
        (unsigned)(1500) initialLookAroundMs, // look around for this time before moving
        (float)(0.5f) walkSpeed,  
        (float)(300.f) distanceExtra, // mm - how much beyond side line to walk
        (float)(300.f) distanceThreshold, // mm
        (float)(10_deg) angleThreshold,
        (float)(400.f) robotRadius, // mm - chose poses outside this to make it easier for obstacle avoidance
        (float)(300.f) smallRadius, // mm - chose poses outside this to make it easier for obstacle avoidance
        (float)(0.05f) kickPower, // Very low kick power to keep ball close (0.05 instead of 0.5)
        
      });

    READS(FieldBall);
    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FrameInfo);
    
    //const Rangea directionPrecision = Rangea(-15_deg, +15_deg);

    bool obstaclesInFront()
    {
      return obstacleSkills.obstaclesInCorridor(Pose2f(), Rangef(-1000.f, 1000.f), theFieldDimensions.yPosLeftSideline * 2);
    }

    Pose2f getFinalTargetPose()
    {
      Pose2f targetPoseOnField(90_deg, theRobotPose.translation.x(), theFieldDimensions.yPosRightSideline - params.distanceExtra);
      return theRobotPose.toRobotCoordinates(targetPoseOnField);
    }

    Pose2f toFieldPose(Angle rotation, float x, float y)
    {
      return theRobotPose.toRobotCoordinates(Pose2f(rotation, x, y));
    }

    // Monitor if ball is within 75cm distance
  
  };

} // Leaderboards
} // RC2025
} // CoroBehaviour
