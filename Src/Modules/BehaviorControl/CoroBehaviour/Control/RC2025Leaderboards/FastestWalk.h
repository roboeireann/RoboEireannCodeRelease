/**
 * @file FastestWalk.h
 *
 * This file implements the RC2025 Fastest Walk Leaderboard Challenge
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
#include "Representations/Modeling/ObstacleModel.h"

namespace CoroBehaviour
{
namespace RC2025
{
namespace Leaderboards
{
  CRBEHAVIOUR(FastestWalkTask)
  {
    CRBEHAVIOUR_INIT(FastestWalkTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // look around briefly (and give time for person pressing button to step away)
      while (getCheckpointDuration() < params.initialLookAroundMs)
      {
        headSkills.lookActive(/* withBall */ false, /* ignoreBall */ true);
        commonSkills.stand();
        CR_YIELD();
      }

      // now decide whether we need to avoid obstacles or just go straight ahead
      if (!obstaclesInFront()) // no obstacles - go straight ahead
      {
        CR_CHECKPOINT(walk_no_obstacles);
        do
        {
          targetPose = getFinalTargetPose();
          headSkills.lookActive(/* withBall */ false, /* ignoreBall */ true);
          walkToPoseTask(targetPose, params.walkSpeed, AVOIDANCE_NONE, /* keepTargetRotation */ true);
          CR_YIELD();
        } while (!commonSkills.isPoseClose(targetPose));

        CR_CHECKPOINT(stand);
        CR_WHILE(true, commonSkills.standLookForward(/* standHigh */ true));
      }
      else // there are obstacles
      {
        prepareWaypoints();
        for (waypointIndex = 0; waypointIndex < waypoints.size(); ++waypointIndex)
        {
          // go to next waypoint
          do
          {
            CR_CHECKPOINT(go_to_next_waypoint);
            ACTGRAPH_FMT("waypoint {}, {}", waypointIndex, waypointName);
            updateWaypoint(waypointIndex);
            headSkills.lookActive(/* withBall */ false, /* ignoreBall */ true);
            walkToPoseTask(targetPose, params.walkSpeed, avoidanceType, keepTargetRotation);
            CR_YIELD();
          } while (!commonSkills.isPoseClose(targetPose));
        }

        // all waypoints done
        CR_CHECKPOINT(stand);
        CR_WHILE(true, commonSkills.standLookForward(/* standHigh */ true));

      }
    }

  private:
    CommonSkills commonSkills  {env};
    MotionSkills motionSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    HeadSkills headSkills {env};
    WalkToPoseTask walkToPoseTask {env};
    ObstacleSkills obstacleSkills {env};

    DEFINES_PARAMS(FastestWalkTask, 
      {, 
        (unsigned)(1500) initialLookAroundMs, // look around for this time before moving
        (float)(1.f) walkSpeed,  
        (float)(300.f) distanceExtra, // mm - how much beyond side line to walk
        (float)(300.f) distanceThreshold, // mm
        (float)(10_deg) angleThreshold,
        (float)(500.f) robotRadius, // mm - chose poses outside this to make it easier for obstacle avoidance
        (float)(350.f) smallRadius, // mm - chose poses outside this to make it easier for obstacle avoidance
      });

      READS(FieldBall);
      READS(RobotPose);
      READS(FieldDimensions);
      READS(ObstacleModel);
      READS(FrameInfo);
      

      Pose2f targetPose;
      size_t waypointIndex;
      std::string waypointName;
      Avoidance avoidanceType = AVOIDANCE_NONE;
      bool keepTargetRotation = true;

      struct Waypoint
      {
        std::string name;
        Pose2f targetPoseOnField;
        Avoidance avoidanceType;
        bool keepTargetRotation;
      };

      std::vector<Waypoint> waypoints;


      void prepareWaypoints()
      {
        waypoints.push_back({"left_of_obstacle1",
                             Pose2f(180_deg, theFieldDimensions.xPosOwnPenaltyArea + 400.f,
                                    theFieldDimensions.yPosLeftPenaltyArea + params.smallRadius),
                             AVOIDANCE_ROUGH, false});
        waypoints.push_back({"left_of_obstacle1_continued",
                             Pose2f(180_deg, theFieldDimensions.xPosOwnPenaltyArea + 400.f,
                                    theFieldDimensions.yPosLeftPenaltyArea - params.smallRadius),
                             AVOIDANCE_ROUGH, false});

        waypoints.push_back(
            {"right_of_obstacle2",
             Pose2f(-90_deg, ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-100.f,
                    theFieldDimensions.yPosLeftGoalArea + params.robotRadius),
             AVOIDANCE_ROUGH, false});
        waypoints.push_back(
            {"behind_obstacle2",
             Pose2f(0_deg, ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-100.f,
                    theFieldDimensions.yPosLeftGoalArea - params.robotRadius),
             AVOIDANCE_ROUGH, true});


        waypoints.push_back({"left_of_obstacle3",
                             Pose2f(-90_deg, theFieldDimensions.xPosOwnPenaltyMark + params.smallRadius,
                                    theFieldDimensions.yPosCenterGoal + params.robotRadius),
                             AVOIDANCE_ROUGH, false});
        waypoints.push_back({"left_of_obstacle3_continued",
                             Pose2f(180_deg, theFieldDimensions.xPosOwnPenaltyMark + params.smallRadius,
                                    theFieldDimensions.yPosCenterGoal - params.robotRadius),
                             AVOIDANCE_ROUGH, false});

        waypoints.push_back(
            {"right_of_obstacle4",
             Pose2f(0_deg, ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-150.f,
                    theFieldDimensions.yPosRightGoalArea + params.robotRadius),
             AVOIDANCE_ROUGH, false});
        waypoints.push_back(
            {"behind_obstacle4",
             Pose2f(0_deg, ((theFieldDimensions.xPosOwnGoalArea + theFieldDimensions.xPosOwnGroundLine) / 2)-150.f,
                    theFieldDimensions.yPosRightGoalArea - params.robotRadius),
             AVOIDANCE_NONE, true});

        waypoints.push_back({"final_target",
                             Pose2f(-90_deg, theRobotPose.translation.x(), theFieldDimensions.yPosRightSideline - params.distanceExtra),
                             AVOIDANCE_AUTO, false});
      }

      void updateWaypoint(size_t i)
      {
        waypointName = waypoints[i].name;
        targetPose = theRobotPose.toRobotCoordinates(waypoints[i].targetPoseOnField);
        avoidanceType = waypoints[i].avoidanceType;
        keepTargetRotation = waypoints[i].keepTargetRotation;
      }

      Pose2f getFinalTargetPose()
      {
        Pose2f targetPoseOnField(-90_deg, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightSideline - params.distanceExtra);
        return theRobotPose.toRobotCoordinates(targetPoseOnField);
      }


      //checks if there are obstacles on the field
      bool obstaclesInFront()
      {   
        return obstacleSkills.obstaclesInCorridor(Pose2f(), Rangef(-1000.f, 1000.f), theFieldDimensions.yPosLeftSideline * 2);
        //  !theObstacleModel.obstacles.empty();
      }

    };
  
} // Leaderboards
} // RC2025
} // CoroBehaviour

