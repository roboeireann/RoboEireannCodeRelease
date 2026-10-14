/**
 * @file: HeadSkills.h
 *
 * Skills that result in various head motions
 * 
 * These skills are adapted from skills in the
 * RE2025/BH2025 code releases to fit the Hybrid behaviour framework
 *
 * @author: Rudi Villing
 */

#include "REHybridBehaviourModule.h"


// ------------------------------------------------------------------------
// stateless helper functions that DO NOT have access to the BehaviourContainer
// ------------------------------------------------------------------------

namespace
{
  void setPanTiltRequest(HeadMotionRequest& theHeadMotionRequest, HeadMotionRequest::CameraControlMode camera,
                         Angle pan, Angle tilt, Angle speed = 60_deg, bool stopAndGoMode = false, bool calibrationMode = false)
  {
    theHeadMotionRequest.mode = calibrationMode ? HeadMotionRequest::calibrationMode : HeadMotionRequest::panTiltMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.pan = pan;
    theHeadMotionRequest.tilt = tilt;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
  }

  void setTargetOnGroundRequest(HeadMotionRequest& theHeadMotionRequest, HeadMotionRequest::CameraControlMode camera,
                                const Vector3f& target, Angle speed)
  {
    theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    theHeadMotionRequest.cameraControlMode = camera;
    theHeadMotionRequest.target = target;
    theHeadMotionRequest.speed = speed;
    theHeadMotionRequest.stopAndGoMode = false;
  }
}

// ------------------------------------------------------------------------
// Tasks
// ------------------------------------------------------------------------

CORO_DEFINE((BehaviourContainer, headPlayDeadTask))
{
  CORO_BEGIN()
  while (true)
  {
    setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, JointAngles::off, JointAngles::off);
    theLibCheck.inc(LibCheck::headMotionRequest);
    CORO_YIELD();
  }
  CORO_END()
}



CORO_DEFINE((BehaviourContainer, lookActiveTask),
       ARGS((bool, withBall),
            (bool, ignoreBall), // ignoreBall overrides withBall - only one should be true
            (bool, onlyOwnBall), // ignore team ball if true
            (bool, fixTilt), // use default fixed tilt value
            (float, slowdownFactor)))
{
  const HeadTarget target = theLibLookActive.calculateHeadTarget(args.withBall, args.ignoreBall, args.onlyOwnBall, args.fixTilt, args.slowdownFactor);

  CORO_BEGIN()
  while (true)
  {
    setPanTiltRequest(theHeadMotionRequest, target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
    theLibCheck.inc(LibCheck::headMotionRequest);
    CORO_YIELD();
  }
  CORO_END()
}



CORO_DEFINE((BehaviourContainer, lookAtAnglesTask),
       ARGS((Angle, pan),
            (Angle, tilt),
            (float, speed),
            (HeadMotionRequest::CameraControlMode, camera),
            (bool, calibrationMode)))
{
  CORO_BEGIN()
  while (true)
  {
    setPanTiltRequest(theHeadMotionRequest, args.camera, args.pan, args.tilt, args.speed, false, args.calibrationMode);
    theLibCheck.inc(LibCheck::headMotionRequest);
    CORO_YIELD();
  }
  CORO_END()
}



HSM_DEFINE((BehaviourContainer, lookAtBallTask),
       ARGS((HeadMotionRequest::CameraControlMode, camera)),
       PARAMS((int, ownBallTimeout, 2000), /**< Use the team ball or look active if the ball hasn't been seen for this time. */
            (int, ownBallDisappearedTimeout, 500), /**< Use the team ball or look active if the ball disappeared for this time. */
            (float, propagateBallTime, 0.2f)))
{
  const bool useOwnEstimate = theFieldBall.ballWasSeen(params.ownBallTimeout) &&
                              theFieldBall.timeSinceBallDisappeared <= params.ownBallDisappearedTimeout;

  HSM_GLOBAL_TRANSITIONS
  {
    if(useOwnEstimate || theTeamBallModel.isValid)
      HSM_GOTO(lookAtBall);
    else
      HSM_GOTO(lookActive);
  }

  HSM_INITIAL_STATE(lookAtBall)
  {
    const Vector2f ballPosition =
        theFieldBall.recentBallPropagatedPositionRelative(params.propagateBallTime, theBallSpecification.friction);
    setTargetOnGroundRequest(theHeadMotionRequest, args.camera,
                              {ballPosition.x(), ballPosition.y(), theBallSpecification.radius}, 180_deg);
    theLibCheck.inc(LibCheck::headMotionRequest);
  }

  HSM_STATE(lookActive)
  {
    const HeadTarget target = theLibLookActive.calculateHeadTarget(false, false, false, false, 1.f);
    setPanTiltRequest(theHeadMotionRequest, target.cameraControlMode, target.pan, target.tilt, target.speed, target.stopAndGoMode);
    theLibCheck.inc(LibCheck::headMotionRequest);
  }
}


CORO_DEFINE((BehaviourContainer, lookAtPointTask),
       ARGS((const Vector3f&, target),
            (HeadMotionRequest::CameraControlMode, camera),
            (Angle, speed)))
{
  CORO_BEGIN()
  while (true)
  {
    setTargetOnGroundRequest(theHeadMotionRequest, args.camera, args.target, args.speed);
    theLibCheck.inc(LibCheck::headMotionRequest);
    CORO_YIELD();
  }
  CORO_END()
}


CORO_DEFINE((BehaviourContainer, lookForwardTask))
{
  CORO_BEGIN()
  while (true)
  {
    setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, 0.f, theBehaviorParameters.defaultLookDownAngle, 150_deg);
    theLibCheck.inc(LibCheck::headMotionRequest);
    CORO_YIELD();
  }
  CORO_END()
}


HSM_DEFINE((BehaviourContainer, lookLeftAndRightTask),
       ARGS((bool, startLeft),
            (Angle, maxPan),
            (Angle, tilt),
            (Angle, speed)))
{
  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if(args.startLeft)
        HSM_GOTO(left);
      else
        HSM_GOTO(right);
    }
  }

  HSM_STATE(left)
  {
    HSM_TRANSITIONS
    {
      if(!theHeadMotionInfo.moving && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] - args.maxPan)) < 5_deg)
        HSM_GOTO(right);
    }

    setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, args.maxPan, args.tilt, args.speed);
    theLibCheck.inc(LibCheck::headMotionRequest);
  }

  HSM_STATE(right)
  {
    HSM_TRANSITIONS
    {
      if(!theHeadMotionInfo.moving && std::abs(Angle::normalize(theJointAngles.angles[Joints::headYaw] + args.maxPan)) < 5_deg)
        HSM_GOTO(left);
    }

    setPanTiltRequest(theHeadMotionRequest, HeadMotionRequest::autoCamera, -args.maxPan, args.tilt, args.speed);
    theLibCheck.inc(LibCheck::headMotionRequest);
  }
}

