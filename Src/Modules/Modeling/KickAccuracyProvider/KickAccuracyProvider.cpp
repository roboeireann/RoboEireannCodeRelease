/**
 * @file KickAccuracyProvider.cpp
 *
 * Implements the sim-only kick accuracy logger.
 *

 */

#include "KickAccuracyProvider.h"
#include "Debugging/Annotation.h"
#include "Debugging/Plot.h"
#include "Streaming/Output.h"

MAKE_MODULE(KickAccuracyProvider);

void KickAccuracyProvider::update(KickAccuracyEvaluation& kickAccuracyEvaluation)
{
  // Hold the last measurement on the plot streams so they stay readable between kicks.
  PLOT("module:KickAccuracyProvider:directionErrorDeg", kickAccuracyEvaluation.lastDirectionError.toDegrees());
  PLOT("module:KickAccuracyProvider:achievedDistance", kickAccuracyEvaluation.lastAchievedDistance);
  PLOT("module:KickAccuracyProvider:distanceError", kickAccuracyEvaluation.lastAchievedDistance - kickAccuracyEvaluation.lastCommandedLength);

  const bool kickPhaseStarted = theMotionInfo.executedPhase == MotionPhase::kick && lastExecutedPhase != MotionPhase::kick;
  lastExecutedPhase = theMotionInfo.executedPhase;

  // The Motion thread snapshots targetDirection/kickLength at kick phase construction, so the
  // last value cached here before the phase switch matches the engine's snapshot up to one frame.
  // Ground-truth pose instead of RobotPose so localization error is excluded from the measurement.
  if(theMotionRequest.motion == MotionRequest::walkToBallAndKick)
  {
    cachedWorldDirection = Angle::normalize(theGroundTruthWorldState.ownPose.rotation + theMotionRequest.targetDirection);
    cachedLength = theMotionRequest.kickLength;
  }

  if(theGroundTruthWorldState.balls.empty())
    return;
  const Vector2f ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
  const float ballSpeed = theGroundTruthWorldState.balls[0].velocity.head<2>().norm();

  // A fall during the kick or while the ball rolls invalidates the measurement (the robot
  // drags or falls onto the ball, or never makes proper contact).
  if(state != idle &&
     (theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen))
  {
    fell = true;
    tainted = true;
  }

  if(state == tracking)
  {
    // Re-acceleration while rolling means someone else touched the ball.
    if(ballSpeed > lastSpeed + taintSpeedJump)
      tainted = true;

    // A heading change while rolling means the ball was deflected (a touch that slows the
    // ball down does not trigger the speed-jump check above).
    if(ballSpeed > headingMinSpeed)
    {
      const Angle heading = theGroundTruthWorldState.balls[0].velocity.head<2>().angle();
      if(!hasReferenceHeading)
      {
        referenceHeading = heading;
        hasReferenceHeading = true;
      }
      else if(std::abs(Angle::normalize(heading - referenceHeading)) > deflectionAngle)
        tainted = true;
    }

    if(ballSpeed < stopSpeedThreshold)
    {
      if(slowSince == 0)
        slowSince = theFrameInfo.time;
    }
    else
      slowSince = 0;

    if(kickPhaseStarted) // We kick again before the ball stopped; close this measurement now.
    {
      tainted = true;
      finalize(ballPosition, kickAccuracyEvaluation);
    }
    else if((slowSince != 0 && theFrameInfo.getTimeSince(slowSince) >= stopHoldTime) ||
            theFrameInfo.getTimeSince(kickStartTime) > maxTrackTime)
      finalize(ballPosition, kickAccuracyEvaluation);

    lastSpeed = ballSpeed;
  }
  else if(state == kicking)
  {
    if((ballPosition - ballStart).norm() > minBallMoveDistance)
    {
      state = tracking;
      lastSpeed = ballSpeed;
      slowSince = 0;
    }
    else if(theFrameInfo.getTimeSince(kickStartTime) > maxWaitForContact && theMotionInfo.executedPhase != MotionPhase::kick)
    {
      OUTPUT_TEXT("KickAccuracy: kick made no ball contact" << (fell ? " (robot fell)" : ""));
      ANNOTATION("KickAccuracyProvider", "kick made no ball contact" << (fell ? " (robot fell)" : ""));
      state = idle;
    }
  }

  if(state == idle && kickPhaseStarted)
  {
    commandedDirection = cachedWorldDirection;
    // Goal shots command FLT_MAX and clears command beyond the policy's normalization cap;
    // the policy saturates at maxCommandLength, so measure against that.
    commandedLength = std::min(cachedLength, maxCommandLength);
    ballStart = ballPosition;
    kickStartTime = theFrameInfo.time;
    tainted = false;
    fell = false;
    hasReferenceHeading = false;
    state = kicking;
  }
}

void KickAccuracyProvider::finalize(const Vector2f& ballRest, KickAccuracyEvaluation& kickAccuracyEvaluation)
{
  const Vector2f delta = ballRest - ballStart;
  const Angle actualDirection = delta.angle();
  const Angle directionError = Angle::normalize(actualDirection - commandedDirection);
  const float achievedDistance = delta.norm();

  ++kickAccuracyEvaluation.numKicksMeasured;
  kickAccuracyEvaluation.lastCommandedDirection = commandedDirection;
  kickAccuracyEvaluation.lastActualDirection = actualDirection;
  kickAccuracyEvaluation.lastDirectionError = directionError;
  kickAccuracyEvaluation.lastCommandedLength = commandedLength;
  kickAccuracyEvaluation.lastAchievedDistance = achievedDistance;
  kickAccuracyEvaluation.lastMeasurementTainted = tainted;
  if(!tainted)
  {
    ++numUntainted;
    directionErrorSum += directionError;
    absDirectionErrorSum += std::abs(directionError);
    distanceErrorSum += achievedDistance - commandedLength;
    kickAccuracyEvaluation.meanDirectionError = Angle(directionErrorSum / numUntainted);
    kickAccuracyEvaluation.meanAbsDirectionError = Angle(absDirectionErrorSum / numUntainted);
    kickAccuracyEvaluation.meanDistanceError = distanceErrorSum / numUntainted;
  }

  OUTPUT_TEXT("KickAccuracy #" << kickAccuracyEvaluation.numKicksMeasured
              << ": cmd " << commandedDirection.toDegrees() << "deg / " << commandedLength
              << "mm, actual " << actualDirection.toDegrees() << "deg / " << achievedDistance
              << "mm, dirErr " << directionError.toDegrees()
              << "deg, distErr " << achievedDistance - commandedLength << "mm"
              << (tainted ? " [TAINTED]" : "") << (fell ? " [FELL]" : "")
              << ", mean dirErr " << kickAccuracyEvaluation.meanDirectionError.toDegrees()
              << "deg, mean |dirErr| " << kickAccuracyEvaluation.meanAbsDirectionError.toDegrees()
              << "deg, mean distErr " << kickAccuracyEvaluation.meanDistanceError
              << "mm (" << numUntainted << " kicks)");
  ANNOTATION("KickAccuracyProvider", "kick #" << kickAccuracyEvaluation.numKicksMeasured
             << " dirErr " << directionError.toDegrees() << "deg, distErr "
             << achievedDistance - commandedLength << "mm" << (tainted ? " [TAINTED]" : "") << (fell ? " [FELL]" : ""));

  state = idle;
}
