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

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"

namespace CoroBehaviour
{


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


} // CoroBehaviour