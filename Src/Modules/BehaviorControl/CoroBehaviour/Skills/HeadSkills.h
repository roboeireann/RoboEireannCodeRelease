/**
 * @file: HeadControl.k
 *
 * skills related to head motions and control
 * (some parts adapted from the BH2019 code release HeadControl.cpp)
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Platform/SystemCall.h"

namespace CoroBehaviour
{

  struct HeadSkills
  {
    HeadSkills(BehaviourEnv& env) : env(env) {}

    // these functions are adapted from the BH2019 code release to our approach
    void lookAtAngles(Angle pan, Angle tilt, Angle speed = 180_deg,
                             HeadMotionRequest::CameraControlMode camera = HeadMotionRequest::autoCamera,
                             bool stopAndGoMode = false, bool calibrationMode = false)
    {
      theHeadMotionRequest.mode =
          calibrationMode ? HeadMotionRequest::calibrationMode : HeadMotionRequest::panTiltMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.pan = pan;
      theHeadMotionRequest.tilt = tilt;
      theHeadMotionRequest.speed = speed;
      theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
      theLibCheck.inc(LibCheck::headMotionRequest);
    }

    void lookAtPoint(const Vector3f &target, Angle speed = 180_deg,
                            HeadMotionRequest::CameraControlMode camera = HeadMotionRequest::autoCamera)
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.target = target;
      theHeadMotionRequest.speed = speed;
      theHeadMotionRequest.stopAndGoMode = false;
      theLibCheck.inc(LibCheck::headMotionRequest);
    }

    /**
     * Move the head to look at interesting points
     * @param withBall Whether the ball must be in the image
     * @param ignoreBall Whether the ball should be completely ignored
     * @param onlyOwnBall Whether to use only the own ball model and not the team ball model
     * @param fixTilt Whether to use a fix tilt or to allow the head control to calculate it
     */
    void lookActive(bool withBall = false, bool ignoreBall = false, bool onlyOwnBall = false,
                           bool fixTilt = false)
    {
      const HeadTarget target =
          theLibLookActive.calculateHeadTarget(withBall, ignoreBall, onlyOwnBall, fixTilt);
      lookAtAngles(target.pan, target.tilt, target.speed, target.cameraControlMode, target.stopAndGoMode);
    }


    /**
     * This skill moves the head so that the ball is focused by one camera.
     * @param mirrored Whether to look at the mirrored (about the center of the field) ball position
     * @param forceOwnEstimate Whether to use only the own ball model and not the team ball model
     */
    void lookAtBall(bool mirrored = false, bool forceOwnEstimate = false)
    {
      const int OWN_BALL_TIMEOUTMS = 2000; /**< LookAtBall will use the team ball or look active if the ball hasn't been seen for this time. */
      const int OWN_BALL_DISAPPEARED_TIMEOUTMS = 500; /**< LookAtBall will use the team ball or look active if the ball disappeared for this time. */

      const bool useOwnEstimate =
          forceOwnEstimate || (theFieldBall.ballWasSeen(OWN_BALL_TIMEOUTMS) &&
                               theFieldBall.timeSinceBallDisappeared <= OWN_BALL_DISAPPEARED_TIMEOUTMS);

      if (useOwnEstimate || theTeamBallModel.isValid)
      {
        const Vector2f ballPosition =        
            useOwnEstimate ? (mirrored ? env.toRobotCoordinates(env.toFieldCoordinates(theBallModel.estimate.position).rotated(pi))
                                       : theBallModel.estimate.position)
                           : (mirrored ? env.toRobotCoordinates(theTeamBallModel.position.rotated(pi))
                                       : env.toRobotCoordinates(theTeamBallModel.position));
                              
        lookAtPoint(Vector3f(ballPosition.x(), ballPosition.y(), theBallSpecification.radius), 180_deg,
                    HeadMotionRequest::autoCamera);
      }
      else
      {
        const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false);
        lookAtAngles(target.pan, target.tilt, target.speed, target.cameraControlMode, target.stopAndGoMode);
      }
    }

    /**
     * This skill moves the head so that the team ball is focused by one camera.
     * @param mirrored Whether to look at the mirrored (about the center of the field) ball position
     */
    void lookAtGlobalBall(bool mirrored = false)
    {
      if (theTeamBallModel.isValid)
      {
        const Vector2f ballPosition = env.toRobotCoordinates(mirrored ? theTeamBallModel.position.rotated(pi) : theTeamBallModel.position);
        lookAtPoint(Vector3f(ballPosition.x(), ballPosition.y(), theBallSpecification.radius), 180_deg,
                    HeadMotionRequest::autoCamera);
      }
      else
      {
        const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false);
        lookAtAngles(target.pan, target.tilt, target.speed, target.cameraControlMode, target.stopAndGoMode);
      }
    }

    /**
     * look straight ahead with the default neck tilt for good near and far view
     */
    void lookForward()
    {
      lookAtAngles(0.f, 0.38f, 150_deg, HeadMotionRequest::autoCamera);
    }

    // based on previous RoboEireann code...

    /**
     * lookDown - usually when the robot is penalized
     */
    void lookDown() 
    { 
      lookAtAngles(0.f, 28_deg, 150_deg, HeadMotionRequest::autoCamera); 
    }

  private:
    BehaviourEnv& env;

    READS(LibCheck);
    READS(LibLookActive);
    READS(BallModel);
    READS(FieldBall);
    READS(TeamBallModel);
    READS(BallSpecification);
    MODIFIES(HeadMotionRequest);
  };

  // ------------------------------------------------------------------------
  // resumable tasks
  // ------------------------------------------------------------------------

  // based on BH2021 HeadControl.cppp
  CRBEHAVIOUR(LookLeftAndRightTask)
  {
    CRBEHAVIOUR_INIT(LookLeftAndRightTask) {}

    void operator()(bool startLeft = true, Angle maxPan = 50_deg, Angle tilt = 23_deg, Angle speed = 100_deg)
    {
      CRBEHAVIOUR_BEGIN();

      lookLeftAndRightSign = startLeft ? 1.f : -1.f;

      CR_WHILE(true, lookLeftRight(maxPan, tilt, speed));
    }

  private:
    int lookLeftAndRightSign;

    HeadSkills headSkills {env};

    READS(JointAngles);
    READS(HeadMotionInfo);


    void lookLeftRight(Angle maxPan, Angle tilt, Angle speed)
    {
      Angle angleToTarget =
          std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - lookLeftAndRightSign * maxPan));

      if (!theHeadMotionInfo.moving && (angleToTarget < 5_deg))
        lookLeftAndRightSign = -lookLeftAndRightSign;

      headSkills.lookAtAngles(lookLeftAndRightSign * maxPan, tilt, speed, HeadMotionRequest::autoCamera);
    }
  };


  CRBEHAVIOUR(ScanSideToSideTask)
  {
    CRBEHAVIOUR_INIT(ScanSideToSideTask) {}

    void operator()(Angle pan = 0.5f, Angle tilt = 0.38f, float speed = 180_deg,
                    HeadMotionRequest::CameraControlMode camera = HeadMotionRequest::autoCamera)
    {
      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(look_left);
      CR_WHILE(getCheckpointDuration() < LOOK_LEFT_MS, headSkills.lookAtAngles(pan, tilt, speed, camera));

      CR_CHECKPOINT(look_center_1);
      CR_WHILE(getCheckpointDuration() < LOOK_CENTER_MS, headSkills.lookAtAngles(0.f, tilt, speed, camera));

      CR_CHECKPOINT(look_right);
      CR_WHILE(getCheckpointDuration() < LOOK_RIGHT_MS, headSkills.lookAtAngles(-pan, tilt, speed, camera));

      CR_CHECKPOINT(look_center_2);
      CR_WHILE(getCheckpointDuration() < LOOK_CENTER_MS, headSkills.lookAtAngles(0.f, tilt, speed, camera));

      // task done, ensure valid output
      headSkills.lookAtAngles(0.f, tilt, speed, camera);
      // mark the task done, it will reset/restart when called again
      CR_EXIT_SUCCESS();
    }

  private:
    const CoroTime LOOK_LEFT_MS = 1000;
    const CoroTime LOOK_RIGHT_MS = 1000;
    const CoroTime LOOK_CENTER_MS = 1000;

    HeadSkills headSkills {env};
  };

} // CoroBehaviour