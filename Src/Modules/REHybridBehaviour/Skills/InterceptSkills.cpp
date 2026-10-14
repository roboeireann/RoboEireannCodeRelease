/**
 * @file InterceptSkills.cpp
 *
 * Intercept behaviour for outfield players.
 *
 * @author Rudi Villing
 */

 #include "REHybridBehaviourModule.h"


// ------------------------------------------------------------------------
// KeyFrameInterceptTask
// ------------------------------------------------------------------------

CORO_DEFINE((BehaviourContainer, keyFrameInterceptTask),
    ARGS((MotionRequest::Dive::Request, interceptMotion), (unsigned, getUpDelay)),
    VARS((MotionRequest::Dive::Request, interceptMotion), (unsigned, getUpDelay), (unsigned, waitForInterceptMs),
        (bool, actionDone)),
    PARAMS((unsigned, waitForGenuflectMs, 2000)))
{
  auto latchParams = [&](MotionRequest::Dive::Request interceptMotionIn, unsigned getUpDelayIn)
  {
    interceptMotion = interceptMotionIn;
    getUpDelay = getUpDelayIn;

    switch (interceptMotion)
    {
    case MotionRequest::Dive::squatArmsBackLeft:
    case MotionRequest::Dive::squatArmsBackRight:
    case MotionRequest::Dive::squatWideArmsBackLeft:
    case MotionRequest::Dive::squatWideArmsBackRight:
      waitForInterceptMs = params.waitForGenuflectMs;
      break;
    default:
      waitForInterceptMs = 0;
    }
  };

  CORO_BEGIN()
  latchParams(args.interceptMotion, args.getUpDelay);

  // execute selected intercept motion

  CORO_STATE(intercept);
  actionDone = false;
  while (!actionDone)
  {
    lookForwardTask();
    actionDone = diveTask({.request = interceptMotion});
    CORO_YIELD();
  }

  // wait for intercept motion to complete

  CORO_STATE(wait_for_intercept_to_complete);
  while (THIS_TASK.taskDuration() < waitForInterceptMs) // time since start of coro, not just checkpoint
  {
    lookForwardTask();
    diveTask({.request = interceptMotion});
    CORO_YIELD();
  }

  // wait for additional get up delay if needed

  CORO_STATE(wait_for_getup_delay);
  while (THIS_TASK.stateDuration() < getUpDelay)
  {
    lookForwardTask();
    diveTask({.request = interceptMotion});
    CORO_YIELD();
  }

  // finally, execute the get up

  CORO_STATE(get_up_after_intercept);
  while (true)
  {
    lookForwardTask();
    if (HBS_SUCCESS(standTask()))
      CORO_SUCCESS();
    else
      CORO_YIELD();
  }

  CORO_END()
}


// ------------------------------------------------------------------------
// InterceptTask
// ------------------------------------------------------------------------

enum class InterceptTaskMode { STAY_AS_IS, WALK, KEYFRAME };

CORO_DEFINE((BehaviourContainer, interceptTask),
    ARGS((unsigned, interceptsEnabled), (bool, aggressive), (unsigned, getUpDelay)),
    VARS(
      (InterceptTaskMode, interceptMode),
      (MotionRequest::Dive::Request, interceptMotion),
      (bool, canSeeBallAfterIntercept),
      (bool, wasInterceptNeeded),
      (unsigned, timeWhenInterceptNeededChanged, 0),
      (float, interceptY),
      (float, interceptYSecs),
      (bool, shouldInterceptY), // should a y intercept be performed
      (bool, isAggressiveY), // is a y intercept only to be performed according if aggressively anticipate interceptions
      (float, interceptX),
      (float, interceptXSecs),
      (bool, shouldInterceptX)
    ),
    PARAMS(
      (Rangef, standRange, {0.f, 30.f}), ///< a ball inside this y-distance can be intercepted by just standing
      (Rangef, genuflectRange, {30.f, 230.f}), ///< a ball inside this y-distance can be intercepted using the genuflect motion
      (unsigned, waitForGenuflectMs, 2000),
      // (unsigned)(300) ballSeenTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef, keyframeTimeRangeSecs, {0.1f, 3.f}), ///< we don't try a keyframe intercept if the ball will pass too quick or too far in the future
      (float, walkXThreshold, 100.f), ///< walk to intercept if the x-intercept is less than this and would hit the side of the feet
      (Angle, sideApproachAngle, 135_deg), ///< consider angles less than this to be side approach (we don't expect angles < 90)
      (unsigned, ballSeenTimeoutMs, 1000), ///< we can only intercept the ball if seen pretty recently
      (int, ballDisappearedTimeoutMs, 500), ///< ball disappeared means it should have been visible in the image but wasn't
      (float, keyframeInterceptY, 230.f), ///< max y at which keyframe intercepts make sense (see genuflect stuff params in InterceptTask)
      (float, walkInterceptSpeedY, 240.f), ///< mm/sec - ambitious so we give it a shot, even if we might not make it
      (float, walkInterceptSpeedX, 180.f), ///< mm/sec - conservative so we don't try for x intercept generally
      (Rangef, interceptYSecsRange, {0.05f, 8.f}), ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
      (Rangef, interceptXSecsRange, {0.05f, 4.f}) ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
    ))
{
  static DECL_TLOGGER_VAR(tlogger, "InterceptTask", TextLogging::DEBUG);

  // static constexpr unsigned DEFENDER_INTERCEPTS = bit(Interception::genuflectStandDefender) | bit(Interception::walk);
  // static constexpr unsigned OUTFIELD_INTERCEPTS = bit(Interception::genuflectStand) | bit(Interception::walk);
  // static constexpr unsigned WALK_INTERCEPTS = bit(Interception::walk);

  
  auto updateInterceptValues = [&](bool aggressive)
  {
    shouldInterceptY = false;
    isAggressiveY = false;
    shouldInterceptX = false;

    // y-intercept first

    interceptY = theFieldInterceptBall.intersectionPositionWithOwnYAxis.y();
    interceptYSecs = theFieldInterceptBall.timeUntilIntersectsOwnYAxis;

    // TODO - not implemented yet for BH2025 framework
    // calculate more aggressive intercept if needed
    // if (aggressive && (interceptYSecs == std::numeric_limits<float>::max()))
    // {
    //   interceptY = theFieldInterceptBall.intersectionPositionWithOwnYAxisAggressive.y();
    //   interceptYSecs = theFieldBall.secsUntilIntersectsOwnYAxisAggressive;
    //   isAggressiveY = true;
    // }

    // is y-intercept feasible/needed

    // FIXME - this logic should also consider whether keyframe intercepts are allowed
    shouldInterceptY = params.interceptYSecsRange.isInside(interceptYSecs);
    if (shouldInterceptY && !aggressive)
      shouldInterceptY =
          (std::abs(interceptY) < std::max(params.keyframeInterceptY, params.walkInterceptSpeedY * interceptYSecs));

    interceptX = theFieldInterceptBall.intersectionPositionWithOwnXAxis.x();
    interceptXSecs = theFieldInterceptBall.timeUntilIntersectsOwnXAxis;

    // is x-intercept feasible/recommended
    // the ball will intercept robot x-axis within allowed time range
    // AND interceptX position is reachable by walking
    shouldInterceptX = !shouldInterceptY && params.interceptXSecsRange.isInside(interceptXSecs);
    if (shouldInterceptX && !aggressive)
      shouldInterceptY = (std::abs(interceptX) < (params.walkInterceptSpeedX * interceptXSecs));
  };

  auto isInterceptNeeded = [&](bool aggressive = false) -> bool
  {
    bool needed = true;

    // check for shortcut answer depending on whether we needed an
    // intercept in the last frame or not
    if (wasInterceptNeeded) // was an intercept needed during the last frame
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenTimeoutMs) ||
          (theFieldBall.timeSinceBallDisappeared > params.ballDisappearedTimeoutMs))
      {
        needed = false;
        TLOGD(tlogger, "isInterceptNeeded: ball timeout seen {} disappeared {}", theFieldBall.timeSinceBallWasSeen,
              theFieldBall.timeSinceBallDisappeared);
      }
    }
    else // no intercept needed last frame
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
      {
        needed = false;
        TLOGV(tlogger, "isInterceptNeeded: ball timeout seen {}", theFieldBall.timeSinceBallWasSeen);
      }
    }

    updateInterceptValues(aggressive);

    if (needed) // now check if really needed
      needed = shouldInterceptY || shouldInterceptX;
    else
      TLOGV(tlogger, "isInterceptNeeded: shouldInterceptY {}, shouldInterceptX {}", shouldInterceptY,
            shouldInterceptX);

    if (needed != wasInterceptNeeded) // did we change from needed to not-needed or vice versa?
    {
      const char *neededStr = needed ? "yes-start" : "no-finish";
      const char *aggrStr = isAggressiveY ? " (aggr)" : "";

      HBS_ANNOTATION("isInterceptNeeded {}: y={:.0f} ySecs={:.1f}{}, x={:.0f} xSecs={:.1f}", neededStr,
                          interceptY, std::min(interceptYSecs, 999.f), aggrStr, interceptX,
                          std::min(interceptXSecs, 999.f));
      TLOGV(tlogger, "isInterceptNeeded {}: y={:.0f} ySecs={:.1f}{}, x={:.0f} xSecs={:.1f}", neededStr, interceptY,
            std::min(interceptYSecs, 999.f), aggrStr, interceptX, std::min(interceptXSecs, 999.f));

      wasInterceptNeeded = needed;
      timeWhenInterceptNeededChanged = theFrameInfo.time;
    }

    return needed;
  };


  auto getInterceptPose = [&]() -> Pose2f
  {
    // if (theFieldInterceptBall.timeUntilIntersectsOwnYAxis < theFieldBall.secsUntilIntersectsOwnXAxis)
    if (shouldInterceptY)
    {
      return Pose2f(0.f, 0.f, interceptY);
    }
    else if (shouldInterceptX)
    {
      // the ball will intercept our x-pos so we should turn partly to face it
      Angle ballAngle = theFieldBall.positionRelative.angle();
      return Pose2f(ballAngle, interceptX - (theRobotDimensions.footLength + 50.f), 0.f);
    }
    else
    {
      return Pose2f(0,0,0);
    }
  };

  auto chooseInterceptMotion = [&](unsigned interceptsEnabled)
  {
    // defaults
    interceptMode = InterceptTaskMode::KEYFRAME;
    canSeeBallAfterIntercept = true;

    bool left = (interceptY >= 0);
    float interceptYDistance = std::fabs(interceptY);
    bool insideKeyframeTimeRange = params.keyframeTimeRangeSecs.isInside(interceptYSecs);

    // bool canIntercept =
    //     (theFieldBall.ballWasSeen(300) && between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f));
    // bool noReactionNeeded = (interceptTime > 10.f) || (interceptDistance > std::max(230.f, interceptTime * 150.f));

    if (params.standRange.isInside(interceptYDistance))
    {
      // if ball is approaching from the side and will hit the side of the feet we want to walk to position
      if ((interceptX < params.walkXThreshold) &&
          (std::abs(theBallModel.estimate.velocity.angle()) < params.sideApproachAngle))
        interceptMode = InterceptTaskMode::WALK;
      else
        interceptMode = InterceptTaskMode::STAY_AS_IS; // continue to stand/sit
    }
    else if (!isAggressiveY && insideKeyframeTimeRange && (interceptsEnabled & bit(Interception::genuflectStand)) &&
              params.genuflectRange.isInside(interceptYDistance))
      // interceptMotion = KeyframeMotionRequest::genuflectStand; // sumo wide stance
      interceptMotion = left ? MotionRequest::Dive::squatArmsBackLeft : MotionRequest::Dive::squatArmsBackRight;
    else if (!isAggressiveY && insideKeyframeTimeRange &&
              (interceptsEnabled & bit(Interception::genuflectStandDefender)) &&
              params.genuflectRange.isInside(interceptYDistance))
      // interceptMotion = KeyframeMotionRequest::genuflectStandDefender; // sumo wide stance
      interceptMotion = left ? MotionRequest::Dive::squatWideArmsBackLeft : MotionRequest::Dive::squatWideArmsBackRight;
    else // walk
      interceptMode = InterceptTaskMode::WALK;
  };

  // --------------------------------------------------------------------------

  CORO_BEGIN()

  // if we already called isInterceptNeeded in this frame then wasInterceptNeeded
  // has already been changed if required
  if (timeWhenInterceptNeededChanged != theFrameInfo.time) 
    wasInterceptNeeded = false;

  while (isInterceptNeeded(args.aggressive))
  {
    // decide which save to make
    chooseInterceptMotion(args.interceptsEnabled);
    
    if (interceptMode == InterceptTaskMode::STAY_AS_IS) // no intercept selected?
    {
      CORO_STATE(stay_as_is);
      lookAtBallTask();
      standTask();

      CORO_YIELD();
    }
    else if (interceptMode == InterceptTaskMode::WALK)
    {
      CORO_STATE(walk_intercept);
      lookAtBallTask();
      walkToPoseTask({.target = getInterceptPose(), .speed = 1.f, .avoidance = AVOIDANCE_ROUGH,
                      .keepTargetRotation = true, .standAtTarget = false});

      CORO_YIELD();
    }
    else // InterceptTaskMode::KEYFRAME
    {
      CORO_STATE(keyframe_intercept);
      CORO_AWAIT(keyFrameInterceptTask({.interceptMotion = interceptMotion, .getUpDelay = args.getUpDelay}));
    }
  }

  TLOGD(tlogger, "interceptTask intercept no longer needed");

  // intercept no longer needed
  CORO_STATE(intercept_no_longer_needed);
  lookAtBallTask();
  if (theMotionInfo.isMotion(MotionPhase::walk))
    walkAtRelativeSpeedTask({.speed = {0.f, 0.f, 0.f}});
  else
    standTask();
  CORO_SUCCESS();

  CORO_END()
}
