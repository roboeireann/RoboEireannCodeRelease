/**
 * A basic behaviour to go to the ball and kick more or less straight ahead
 */

#include "REHybridBehaviourModule.h"


HSM_DEFINE((BehaviourContainer, testKicksTask),
  LOADS_PARAMS((unsigned, someTimeout, 47), // just testing config file loading
            (Vector2f, somePoint, {2.f, -2.f})))
{
  HSM_INITIAL_STATE(notPlaying)
  {
    HSM_TRANSITIONS
    {
      if (theGameState.isPlaying())
        HSM_GOTO(searchForBall);
    }
  
    THIS_TASK.traceContext("someTimeout={}", params.someTimeout);
    THIS_TASK.traceContext("somePoint={}", params.somePoint);

    lookForwardTask();
    standTask();
  }

  HSM_STATE(searchForBall)
  {
    HSM_TRANSITIONS
    {
      if (!theGameState.isPlaying())
        HSM_GOTO(notPlaying);
      else if (theFieldBall.ballWasSeen(2000))
        HSM_GOTO(playBall);
    }

    // search behaviour
    testKicksSearchTask();
  }

  HSM_STATE(playBall)
  {
    HSM_TRANSITIONS
    {
      if (!theGameState.isPlaying())
        HSM_GOTO(notPlaying);
      else if (!theFieldBall.ballWasSeen(4000))
        HSM_GOTO(searchForBall);
    }

    // lookAtBallTask();
    // standTask({.high = true});

    // play the ball
    testKicksGoToBallAndKickTask();
  }
}


CORO_DEFINE((BehaviourContainer, testKicksSearchTask),
    PARAMS((unsigned, leftAndRightMs, 2000), /* assuming leftRight is 100_deg/sec */
        (Angle, turnSpeed, 75_deg)))
{
  CORO_BEGIN()
  CORO_STATE(leftAndRight);
  CORO_WHILE(THIS_TASK.stateDuration() < params.leftAndRightMs, { lookLeftAndRightTask(); standTask(); });
 
  CORO_STATE(turn);
  while (true)
  {
    lookAtAnglesTask({.pan = 0, .tilt = 49_deg});
    walkAtAbsoluteSpeedTask({.speed = {params.turnSpeed,0.f,0.f}});
    CORO_YIELD();
  }
  CORO_END()
}


CORO_DEFINE((BehaviourContainer, testKicksGoToBallAndKickTask),
    VARS((unsigned, lastTimeBallWasFar, 0),
        (bool, kickWithLeft, true)),
    PARAMS((int, ballSeenTimeout, 2000),
        (int, minKickInterval, 1000),
        (int, ballCloseTimeout, 1000),
        (float, ballDistanceClose, 300.f),
        (float, positionHysteresis, 50.f),
        (Vector2f, waitPosition, {-200.f, 0})))
{
  if (theBallModel.estimate.position.squaredNorm() > sqr(params.ballDistanceClose))
    lastTimeBallWasFar = theFrameInfo.time;

  const bool ballReliable = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < params.ballSeenTimeout;
  const bool ballKickable = theFrameInfo.getTimeSince(lastTimeBallWasFar) > params.ballCloseTimeout &&
      theBallModel.timeWhenLastSeen > theMotionInfo.lastKickTimestamp + params.minKickInterval;
  const bool shouldKick = ballReliable && ballKickable;

  if (kickWithLeft)
    kickWithLeft = theFieldBall.positionRelative.y() < -params.positionHysteresis; // switch to right if needed
  else // already on right
    kickWithLeft = theFieldBall.positionRelative.y() < params.positionHysteresis; // switch to left if needed

  const float yOffset = kickWithLeft ? -theRobotDimensions.yHipOffset : theRobotDimensions.yHipOffset;

  THIS_TASK.traceContext("ballReliable={}", ballReliable);
  THIS_TASK.traceContext("ballKickable={}", ballKickable);
  THIS_TASK.traceContext("shouldKick={}", shouldKick);
  THIS_TASK.traceContext("kickWithLeft={}", kickWithLeft);

  CORO_BEGIN()
  while (true)
  {
    CORO_STATE(goToBallPosition);
    while (!shouldKick || THIS_TASK.stateDuration() < 2000)
    {
      lookAtBallTask();
      walkToPoseBasicTask({.target = {theBallModel.estimate.position + params.waitPosition + Vector2f(0.f, yOffset)},
                      .speed = {0.5f, 0.5f, 0.5f}});
      CORO_YIELD();
    }

    CORO_STATE(kickTheBall);
    while (shouldKick)
    {
      lookAtBallTask();
      walkToBallAndKickTask({.targetDirection = 0_deg, .kickType = KickInfo::standRLK1KickLongLeft});
      CORO_YIELD();
    }
  }
  CORO_END()
}
