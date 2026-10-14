/**
 * @file: GotoSkills.h
 *
 * Basic (fairly low level) motion related skills (including stand, walking).
 * Generally there is little intelligence in handling the request.
 * For more intelligent walking, see the GotoSkills instead.
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"

namespace RE
{

  // struct GotoSkills
  // {
    // ------------------------------------------------------------------------
    // stateless functions (i.e. not tasks)
    // ------------------------------------------------------------------------

    /**
     * go to a target point relative to the robot exactly and adjust the
     * walk as necessary (e.g. turning early in the walk if needed and
     * turning late in the walk to align to the requested orientation)
     */
    // static bool gotoTarget(BehaviourEnv &env, float speed, const Pose2f &target)
    // {
    //   // const float DIST_BEFORE_FINAL_ROTATE_MM = 200.0f;
    //   // const Angle ANGLE_NEEDS_INITIAL_ROTATE = 60_deg;

    //   Pose2f walkSpeed(speed, speed, speed);

    //   auto obstacleAvoidance =
    //       env.theLibWalk.calcObstacleAvoidance(target, /* rough: */ true, /* disableObstacleAvoidance: */ false);

    //   return MotionSkills::walkToPose(env, target, walkSpeed, obstacleAvoidance, /* keepTargetRotation: */ false);

      // if (targetRel.translation.norm() > DIST_BEFORE_FINAL_ROTATE_MM)
      // {
      //     Angle pathAngle = targetRel.translation.angle();

      //     if (fabs(pathAngle) > ANGLE_NEEDS_INITIAL_ROTATE)
      //     {
      //         // fprintf(stderr, "gotoTargetRel: initial_rotate: pathAngle=%f\n", pathAngle.toDegrees());

      //         // rotate to direction of walking
      //         return MotionSkills::walkToTargetRel(env, walkSpeed, pathAngle);
      //     }
      //     else
      //     {
      //         // fprintf(stderr, "gotoTargetRel: walk: pathAngle=%f, x,y = %f, %f\n",
      //         //         pathAngle.toDegrees(), targetRel.translation.x(), targetRel.translation.y());

      //         // // walk towards target point, but ignore target rotation until we get close
      //         // return MotionSkills::walkToTargetRel(env, walkSpeed, Pose2f(pathAngle, targetRel.translation));
      //         return MotionSkills::walkAtRelativeSpeed(env, Pose2f(Rangea::OneRange().limit(pathAngle), speed, 0));
      //     }
      // }
      // else
      // {
      //     // we're close to the target point so rotate to final rotation during
      //     // last few steps
      //     return MotionSkills::walkToTargetRel(env, walkSpeed, targetRel);
      // }
    // }
  // };


  // =====================================================================

  /**
   * Go to a pose and if the pose changes wait for a short delay before
   * adjusting so that the robot is not constantly fidgeting. If the delay
   * is zero it will just adjust constantly as needed.
   * 
   * This is a walk task only. You need to manage the head movement elsewhere.
   */
  CRBEHAVIOUR(WalkToPoseAndStandTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseAndStandTask) {}

    void operator()(Pose2f target, float walkSpeed = 1.f, unsigned fineTuneDurationMs = 2000,
                    unsigned minOutOfPositionDurationMs = 2000, float distanceThreshold = 100.f /*mm*/,
                    Angle angleThreshold = 10_deg, float outOfPositionDistance = 200.f /*mm*/,
                    Angle outOfPositionAngle = 20_deg)
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walk_to_pose);
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          walkToPoseAutoAvoidanceTask(target, Pose2f(walkSpeed, walkSpeed, walkSpeed),
                                                        /* keepTargetRotation */ false,
                                                        distanceThreshold, angleThreshold);
          CR_YIELD();
        }

        CR_CHECKPOINT(fine_tune_pose);
        while (getCheckpointDuration() < fineTuneDurationMs)
        {
          walkToPoseAutoAvoidanceTask(target, Pose2f(walkSpeed, walkSpeed, walkSpeed),
                                                         /* keepTargetRotation */ false,
                                                         distanceThreshold, angleThreshold);
          CR_YIELD();                                            
        }

        if (!outOfPosition(target, outOfPositionDistance, outOfPositionAngle)) // double check before standing
        {
          CR_CHECKPOINT(stand_at_pose);
          while (!outOfPosition(target, outOfPositionDistance, outOfPositionAngle) ||
                 (getCheckpointDuration() < minOutOfPositionDurationMs))
          {
            commonSkills.stand();
            CR_YIELD();
          }
        }
      }
    }

  private:
    READS(RobotPose);

    CommonSkills commonSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    bool outOfPosition(const Pose2f& target, float outOfPositionDistance, Angle outOfPositionAngle)
    {
      return !commonSkills.isPoseClose(target, outOfPositionDistance, outOfPositionAngle);
    }
  };


} // namespace RE