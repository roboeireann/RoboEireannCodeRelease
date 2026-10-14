/**
 * @file: MotionSkills.h
 *
 * Skills that result in walk/stand/unstiff motions
 * 
 * These skills are adapted from various walk/stand related skills in the
 * RE2025/BH2025 code releases to fit the Hybrid behaviour framework
 *
 * @author: Rudi Villing
 */

#include "REHybridBehaviourModule.h"





HSM_DEFINE((BehaviourContainer, playDeadTask))
{
  theMotionRequest.motion = MotionRequest::playDead;
  theMotionRequest.energySavingWalk = false;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (theMotionInfo.executedPhase == MotionPhase::playDead)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}


HSM_DEFINE((BehaviourContainer, prepareTask))
{
  theMotionRequest.motion = MotionRequest::prepare;
  theMotionRequest.energySavingWalk = false;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (theMotionInfo.executedPhase == MotionPhase::prepare)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}


HSM_DEFINE((BehaviourContainer, standTask),
       ARGS((bool, high),
            (bool, energySavingWalk)))
{
  theMotionRequest.motion = MotionRequest::stand;
  theMotionRequest.standHigh = args.high;
  theMotionRequest.energySavingWalk = args.energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (theMotionInfo.executedPhase == MotionPhase::stand)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}



HSM_DEFINE((BehaviourContainer, diveTask),
    ARGS((MotionRequest::Dive::Request, request)))
{
  theMotionRequest.motion = MotionRequest::dive;
  theMotionRequest.diveRequest = args.request;
  theMotionRequest.energySavingWalk = false;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      const KeyframeMotionRequest keyframeMotionRequest = KeyframeMotionRequest::fromDiveRequest(args.request);
      if (theMotionInfo.isKeyframeMotion(keyframeMotionRequest.keyframeMotion, keyframeMotionRequest.mirror))
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}



HSM_DEFINE((BehaviourContainer, walkAtAbsoluteSpeedTask),
       ARGS((const Pose2f&, speed),
            (bool, energySavingWalk)))
{
  theMotionRequest.motion = MotionRequest::walkAtAbsoluteSpeed;
  theMotionRequest.walkSpeed = args.speed;
  theMotionRequest.energySavingWalk = args.energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (theMotionInfo.executedPhase == MotionPhase::walk)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}


HSM_DEFINE((BehaviourContainer, walkAtRelativeSpeedTask),
       ARGS((const Pose2f&, speed),
            (bool, energySavingWalk)))
{
  theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
  theMotionRequest.walkSpeed = args.speed;
  theMotionRequest.energySavingWalk = args.energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if(theMotionInfo.executedPhase == MotionPhase::walk)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}


CORO_DEFINE((BehaviourContainer, walkToBallAndKickTask),
       ARGS((Angle, targetDirection),
            (KickInfo::KickType, kickType),
            (KickPrecision, alignPrecisely),
            (float, kickLength),
            (const Pose2f&, speed),
            (const MotionRequest::ObstacleAvoidance&, obstacleAvoidance),
            (PreStepType, preStepType),
            (bool, turnKickAllowed),
            (bool, shiftTurnKickPose),
            (const Rangea&, directionPrecision)),
       VARS((unsigned, lastKickTimestamp, 0))) /**< The value of the kick timestamp in the previous frame. */
{
  CORO_BEGIN()
  lastKickTimestamp = theMotionInfo.lastKickTimestamp;
  while (true)
  {
    theMotionRequest.motion = MotionRequest::walkToBallAndKick;
    theMotionRequest.walkSpeed = args.speed;
    theMotionRequest.obstacleAvoidance = args.obstacleAvoidance;
    theMotionRequest.targetDirection = args.targetDirection;
    theMotionRequest.directionPrecision = args.directionPrecision;
    theMotionRequest.kickType = args.kickType;
    theMotionRequest.kickLength = args.kickLength;
    theMotionRequest.alignPrecisely = args.alignPrecisely;
    theMotionRequest.preStepType = args.preStepType;
    theMotionRequest.turnKickAllowed = args.turnKickAllowed;
    theMotionRequest.shiftTurnKickPose = args.shiftTurnKickPose;
    theMotionRequest.energySavingWalk = false;
    theLibCheck.inc(LibCheck::motionRequest);

    if (theMotionInfo.lastKickTimestamp != lastKickTimestamp)
    {
      lastKickTimestamp = theMotionInfo.lastKickTimestamp;
      CORO_YIELD_SUCCESS();
    }
    else
      CORO_YIELD();
  }
  CORO_END()
}


HSM_DEFINE((BehaviourContainer, walkToPoseBasicTask),
       ARGS((const Pose2f&, target),
            (const Pose2f&, speed),
            (const MotionRequest::ObstacleAvoidance&, obstacleAvoidance),
            (bool, keepTargetRotation),
            (const std::optional<Vector2f>&, targetOfInterest),
            (SideWalkingRequest::SideWalkingRequest, sideWalkingRequest),
            (bool, energySavingWalk)))
{
  theMotionRequest.motion = MotionRequest::walkToPose;
  theMotionRequest.walkTarget = args.target;
  theMotionRequest.walkSpeed = args.speed;
  theMotionRequest.keepTargetRotation = args.keepTargetRotation;
  theMotionRequest.obstacleAvoidance = args.obstacleAvoidance;
  theMotionRequest.targetOfInterest = args.targetOfInterest;
  theMotionRequest.sideWalkingRequest = args.sideWalkingRequest;
  theMotionRequest.energySavingWalk = args.energySavingWalk;
  theLibCheck.inc(LibCheck::motionRequest);

  HSM_INITIAL_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (theMotionInfo.executedPhase == MotionPhase::walk)
        HSM_GOTO(done);
    }
  }

  HSM_SUCCESS_STATE(done) {}
}



CORO_DEFINE((BehaviourContainer, walkToPoseTask),
    ARGS((const Pose2f &, target), (float, speed), (Avoidance, avoidance),
        (bool, keepTargetRotation), (bool, standAtTarget),
        (Rangef, positionThreshold), (Rangea, angleThreshold)),
    VARS((Comparatorf, activePathPlannerComparator, {0.f, 0.f, false, true})),
    PARAMS((HystThresholdsf, activePathPlannerThresholds, {900.f, 1000.f}),  
      (unsigned, avoidanceTimeRough, 1000),
      (unsigned, avoidanceTime, 3000),
      (unsigned, standAfterTime, 500), // stand after this much time has elapsed near target
      (unsigned, successDelay, 100))
    )
{
  // static DECL_TLOGGER_VAR(tlogger, "WalkToPoseTask", TextLogging::INFO);

  const Pose2f speedPose = Pose2f(args.speed, args.speed, args.speed);
  const float targetDistance = args.target.translation.norm();
  const bool autoAvoidance = (args.avoidance == AVOIDANCE_AUTO); 

  const bool footContact = (autoAvoidance &&
                            (theFrameInfo.getTimeSince(theFootBumperState.status[Legs::left].lastContact) < 400 ||
                              theFrameInfo.getTimeSince(theFootBumperState.status[Legs::right].lastContact) < 400));
  const bool leftArmContact =
      (autoAvoidance && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::left].timeOfLastContact) < 800));
  const bool rightArmContact =
      (autoAvoidance && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::right].timeOfLastContact) < 800));

  const bool obstacleContacted = footContact || leftArmContact || rightArmContact;

  const bool outsideOuterTargetThreshold =
      (targetDistance > args.positionThreshold.max) || (fabs(args.target.rotation) > args.angleThreshold.max);
  const bool insideInnerTargetThreshold =
      (targetDistance < args.positionThreshold.min) && (fabs(args.target.rotation) < args.angleThreshold.min);

  // init VAR if needed
  if (activePathPlannerComparator.lowerThreshold == 0.f && activePathPlannerComparator.upperThreshold == 0.f)
    activePathPlannerComparator.resetThresholds(params.activePathPlannerThresholds);

  activePathPlannerComparator.update(targetDistance);
  const bool walkFarNeeded = autoAvoidance && activePathPlannerComparator.state;


  auto getFarAvoidance = [&](const Pose2f &target, const Pose2f& speed) -> MotionRequest::ObstacleAvoidance
  {
    return thePathPlanner.plan(theRobotPose.toFieldCoordinates(target), speed);
  };

  auto getNearAvoidance = [&](const Pose2f &target, Avoidance avoidance) -> MotionRequest::ObstacleAvoidance
  {
    return theLibWalk.calcObstacleAvoidance(target, /* rough: */ avoidance != AVOIDANCE_AUTO,
                                            /* disableObstacleAvoidance */ avoidance == AVOIDANCE_NONE,
                                            /* toBall */ avoidance != AVOIDANCE_AUTO);
  };


  CORO_BEGIN()

  while (true)
  {
    while (walkFarNeeded && !obstacleContacted)
    {
      CORO_STATE(walkFar);
      walkToPoseBasicTask({.target = args.target,
                           .speed = speedPose,
                           .obstacleAvoidance = getFarAvoidance(args.target, speedPose),
                           .keepTargetRotation = args.keepTargetRotation});
      THIS_TASK.trace("target: {}", args.target);
      THIS_TASK.trace("keepTargetRotation: {}", args.keepTargetRotation);
      CORO_YIELD();
    }

    while (!walkFarNeeded && !insideInnerTargetThreshold && !obstacleContacted)
    {
      CORO_STATE(walkNear);
      walkToPoseBasicTask({.target = args.target,
                           .speed = speedPose,
                           .obstacleAvoidance = getNearAvoidance(args.target, args.avoidance),
                           .keepTargetRotation = args.keepTargetRotation});
      THIS_TASK.trace("target: {}", args.target);
      THIS_TASK.trace("keepTargetRotation: {}", args.keepTargetRotation);
      CORO_YIELD();
    }

    // by definition insideTargetInnerThreshold implies !walkFarNeeded, so no need to check explicitly
    if (insideInnerTargetThreshold && !obstacleContacted)
    {
      CORO_STATE(reachingDestination);
      while (!outsideOuterTargetThreshold && !obstacleContacted)
      {
        // Note that the robot may already be standing in the right place, in which case we can just stand and exit
        if ((args.standAtTarget && (theMotionInfo.isMotion(MotionPhase::stand))) ||
            (THIS_TASK.stateDuration() > params.standAfterTime))
        {
          standTask();
          THIS_TASK.trace("target: {}", args.target);
          THIS_TASK.trace("keepTargetRotation: {}", args.keepTargetRotation);
          CORO_YIELD(); // if we specified stand at target we'll just do that rather than end the task
        }
        else // don't stand yet
        {
          walkToPoseBasicTask({.target = args.target,
                           .speed = speedPose,
                           .obstacleAvoidance = getNearAvoidance(args.target, args.avoidance),
                           .keepTargetRotation = args.keepTargetRotation});
          THIS_TASK.trace("target: {}", args.target);
          THIS_TASK.trace("keepTargetRotation: {}", args.keepTargetRotation);
          THIS_TASK.trace("standAtTarget: {}", args.standAtTarget);

          // after allowing a little time, we seem to have reached the target.
          // Since we don't stand, we report success to our caller
          if (!args.standAtTarget && (THIS_TASK.stateDuration() > params.successDelay))
            CORO_SUCCESS();
          else
            CORO_YIELD();
        }            
      }
    }

    if (footContact)
    {
      CORO_STATE(footContactBackUp);
      while (THIS_TASK.stateDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
      {
        walkAtRelativeSpeedTask({.speed = {0.f, -1.f, 0.f}}); // backwards based on robot orientation
        CORO_YIELD();
      }
    }
    else if (leftArmContact)
    {
      CORO_STATE(leftContactMoveRight);
      while (THIS_TASK.stateDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
      {
        walkAtRelativeSpeedTask({.speed = {0.f, 0.f, -1.f}}); // right based on robot orientation
        CORO_YIELD();
      }
    }
    else if (rightArmContact)
    {
      CORO_STATE(righContactMoveLeft);
      while (THIS_TASK.stateDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
      {
        walkAtRelativeSpeedTask({.speed = {0.f, 0.f, 1.f}}); // left based on robot orientation
        CORO_YIELD();
      }
    }
  }

  CORO_END()
}


// ------------------------------------------------------------------------
// stateless helper functions (not tasks)
// ------------------------------------------------------------------------

