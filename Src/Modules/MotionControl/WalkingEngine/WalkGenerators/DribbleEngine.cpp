/**
 * @file DribbleEngine.cpp
 *
 * This file implements a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 */

#include "DribbleEngine.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Framework/Settings.h"
#include "Debugging/Annotation.h"
#include "Math/BHMath.h"
#include "Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/Transformation.h"

MAKE_MODULE(DribbleEngine);

using namespace Motion::Transformation;

void DribbleEngine::update(DribbleGenerator& dribbleGenerator)
{
  DECLARE_DEBUG_RESPONSE("module:DribbleEngine:dribbleInWalkKick");
  dribbleGenerator.createPhase = [this](const MotionRequest& motionRequest, const MotionPhase& lastPhase)
  {
    const bool isLeftPhase = theWalkGenerator.isNextLeftPhase(lastPhase, Pose2f(0.f, motionRequest.ballEstimate.position));
    const Pose2f scsCognition = getTransformationToZeroStep(theTorsoMatrix, theRobotModel, theWalkStepData.yHipOffset, motionRequest.odometryData, theOdometryDataPreview, isLeftPhase);

    const Rangef ttrbRange(1.f, 2.f);
    const float timeToReachBall = ttrbRange.limit(std::max(std::abs(motionRequest.ballEstimate.position.x()) / theWalkingEngineOutput.maxSpeed.translation.x(), std::abs(motionRequest.ballEstimate.position.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));

    const Vector2f perceivedBallPosition = scsCognition * motionRequest.ballEstimate.position;
    const float ballVelocityFactor = mapToRange(perceivedBallPosition.norm(), ballVelocityInterpolationRange.min, ballVelocityInterpolationRange.max, 0.f, 1.f);
    const float ballVelocity = motionRequest.ballEstimate.velocity.norm();
    const float useVelocityLength = (1.f - ballVelocityFactor) * std::min(ballVelocity, minBallVelocityCloseRange) + ballVelocityFactor * ballVelocity;
    const Vector2f ballSCS = scsCognition * BallPhysics::propagateBallPosition(motionRequest.ballEstimate.position, motionRequest.ballEstimate.velocity.normalized(useVelocityLength), timeToReachBall, theBallSpecification.friction);

    const Angle directionSCS = Angle::normalize(scsCognition.rotation + motionRequest.targetDirection);

    // Decide the dribble foot once per phase, so the kick-trigger check below and the pose the
    // robot walks to (calcBasePose) always target the same foot pocket.
    updateDribbleSign(ballSCS, directionSCS);

    if(theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen) < 3000.f)
    {
      if(Global::getSettings().robotType == Settings::k1)
      {
        const KickInfo::KickType kickType = lastSign > 0.f ? KickInfo::standRLK1KickLongRight : KickInfo::standRLK1KickLongLeft;
        const Vector2f expectedBallInRobot = -theKickInfo[kickType].ballOffset;
        // Trigger on the perceived ball
        bool isInPositionForKick = (perceivedBallPosition - expectedBallInRobot).norm() < k1KickPositionTolerance;
        const Angle directionError = -Angle::normalize(directionSCS + theKickInfo[kickType].rotationOffset);
        const Rangea allowedDirectionError(
          std::max<Angle>(-k1KickDirectionPrecisionCap, std::min<Angle>(-k1KickDirectionTolerance, motionRequest.directionPrecision.min)),
          std::min<Angle>(k1KickDirectionPrecisionCap, std::max<Angle>(k1KickDirectionTolerance, motionRequest.directionPrecision.max)));
        isInPositionForKick &= allowedDirectionError.isInside(directionError);
        isInPositionForKick &= lastPhase.type == MotionPhase::stand || lastPhase.type == MotionPhase::walk; // prevent kicking from potential unstable phases
        isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
        // velocity.norm() does not need to be transformed to another coordinate system.
        isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < maxBallVelocity;

        if(isInPositionForKick)
        {
          // Fallback: once positioned behind the ball (same positioning as the kick version), walk
          // through the ball instead of kicking it.
          if(walkThroughBallInsteadOfKick)
            return createWalkThroughBallPhase(motionRequest, ballSCS, directionSCS, isLeftPhase, lastPhase);

          KickRequest kr;
          kr.kickMotionType = theKickInfo[kickType].kickMotionType;
          kr.mirror = theKickInfo[kickType].mirror;
          kr.armsBackFix = false;
          kr.calcDynPoints = [](int) -> std::vector<DynPoint> { return {}; };
          auto kickPhase = theKickGenerator.createPhase(kr, lastPhase);
          if(kickPhase)
          {
            kickPhase->kickType = kickType;
            return kickPhase;
          }
          else
            ANNOTATION("DribbleEngine", "Creating Kick Phase of Type " << TypeRegistry::getEnumName(kickType) << " in DribbleEngine returned an empty kick!");
        }
      }
      else
      {
        // check forward kick that is faster to execute
        Legs::Leg kickLeg = ballSCS.y() > 0.f ? Legs::left : Legs::right;

        WalkKickVariant walkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, motionRequest.alignPrecisely, motionRequest.kickLength, motionRequest.targetDirection, false);
        bool isInPositionForKick = theWalkKickGenerator.canStart(walkKickVariant, lastPhase, motionRequest.directionPrecision, motionRequest.preStepType, motionRequest.turnKickAllowed);

        // check forward kick that is slower but executable on the next step
        if(!isInPositionForKick)
        {
          kickLeg = ballSCS.y() < 0.f ? Legs::left : Legs::right;
          walkKickVariant = WalkKickVariant(kickLeg == Legs::left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight, WalkKicks::Type::forward, kickLeg, motionRequest.alignPrecisely, motionRequest.kickLength, motionRequest.targetDirection, false);
          isInPositionForKick = theWalkKickGenerator.canStart(walkKickVariant, lastPhase, motionRequest.directionPrecision, motionRequest.preStepType, motionRequest.turnKickAllowed);
        }

        isInPositionForKick &= motionRequest.ballTimeWhenLastSeen >= theMotionInfo.lastKickTimestamp;
        // velocity.norm() does not need to be transformed to another coordinate system.
        isInPositionForKick &= motionRequest.ballEstimate.velocity.norm() < maxBallVelocity;

        if(isInPositionForKick)
        {
          auto kickPhase = theWalkKickGenerator.createPhase(walkKickVariant, lastPhase, false);
          if(kickPhase)
          {
            kickPhase->kickType = walkKickVariant.kickType;
            return kickPhase;
          }
          else
            ANNOTATION("DribbleEngine", "Creating Kick Phase of Type " << TypeRegistry::getEnumName(walkKickVariant.kickType) << " in WalkToBallAndKickEngine returned an empty kick!");
        }
      }
    }
    MotionRequest::ObstacleAvoidance obstacleAvoidanceSCS = motionRequest.obstacleAvoidance;
    obstacleAvoidanceSCS.avoidance.rotate(scsCognition.rotation);
    for(auto& segment : obstacleAvoidanceSCS.path)
      segment.obstacle.center = scsCognition * segment.obstacle.center;

    Vector2f newBallSCS = ballSCS;
    calcInterceptionPosition(motionRequest, newBallSCS, perceivedBallPosition, scsCognition);
    return theWalkToBallGenerator.createPhase(calcBasePose(newBallSCS, directionSCS, lastSign, motionRequest.turnKickAllowed && motionRequest.preStepType != PreStepType::notAllowed), newBallSCS, theFrameInfo.getTimeSince(motionRequest.ballTimeWhenLastSeen), scsCognition, obstacleAvoidanceSCS, motionRequest.walkSpeed, lastPhase);
  };
}

void DribbleEngine::updateDribbleSign(const Vector2f& ballInSCS, Angle directionInSCS)
{
  // Determine the foot which should be behind the ball depending on whether the robot is left or right of the line from the ball to the target.
  // -1 = dribble with left foot, 1 = dribble with right foot
  const Angle angleDiff = Angle::normalize(directionInSCS - ballInSCS.angle());
  if(std::abs(angleDiff) > redecideSignThreshold)
    lastSign = static_cast<float>(sgnPos(angleDiff));
}

Pose2f DribbleEngine::calcBasePose(const Vector2f& ballInSCS, Angle directionInSCS, float sign, const bool turnKickAllowed) const
{
  Pose2f kickPose(directionInSCS, ballInSCS);

  if(Global::getSettings().robotType == Settings::k1)
  {
    // [K1] Walk to the stance expected by the RL kick instead of the walk kick's stance.
    const KickInfo::KickType kickType = sign > 0.f ? KickInfo::standRLK1KickLongRight : KickInfo::standRLK1KickLongLeft;
    kickPose.rotate(theKickInfo[kickType].rotationOffset);
    kickPose.translate(theKickInfo[kickType].ballOffset);
    return kickPose;
  }

  KickInfo::KickType forwardKick = sign > 0.f ? KickInfo::walkForwardsRight : KickInfo::walkForwardsLeft;
  KickInfo::KickType turnKick = forwardKick == KickInfo::walkForwardsLeft ? KickInfo::walkTurnLeftFootToRight : KickInfo::walkTurnRightFootToLeft;
  const Angle useDirection = turnKickAllowed ? directionInSCS : 0_deg;
  const float factor = Global::getSettings().robotType != Settings::nao ? 0.f : Rangef::ZeroOneRange().limit(useDirection / -theKickInfo[turnKick].rotationOffset);
  kickPose.rotate(theKickInfo[forwardKick].rotationOffset * (1.f - factor) + theKickInfo[turnKick].rotationOffset * factor);
  kickPose.translate(theKickInfo[forwardKick].ballOffset * (1.f - factor) + theKickInfo[turnKick].ballOffset * factor);
  return kickPose;
}

std::unique_ptr<MotionPhase> DribbleEngine::createWalkThroughBallPhase(const MotionRequest& motionRequest, const Vector2f& ballInSCS,
                                                                      const Angle directionInSCS, const bool isLeftPhase,
                                                                      const MotionPhase& lastPhase) const
{
  // Aim at a point beyond the ball along the dribble direction, so the robot keeps walking forward
  // through the ball instead of stopping at it. Because this is recomputed every step with a fresh
  // ball estimate, the target stays ahead of the ball and results in a continuous push.
  const Vector2f target = ballInSCS + Vector2f(walkThroughBallReach, 0.f).rotated(directionInSCS);

  Pose2f step;
  step.rotation = theWalkGenerator.getRotationRange(isLeftPhase, motionRequest.walkSpeed).clamped(directionInSCS);

  std::vector<Vector2f> translationPolygon;
  std::vector<Vector2f> translationPolygonNoCenter;
  theWalkGenerator.getTranslationPolygon(isLeftPhase, step.rotation, lastPhase, motionRequest.walkSpeed,
                                         translationPolygon, translationPolygonNoCenter, false, false);

  if(Geometry::isPointInsideConvexPolygon(translationPolygon.data(), static_cast<int>(translationPolygon.size()), target))
    step.translation = target;
  else
  {
    // Clip the requested step to the largest reachable step in the target direction.
    Vector2f intersection;
    if(target.squaredNorm() > 0.f &&
       Geometry::getIntersectionOfLineAndConvexPolygon(translationPolygon, Geometry::Line(Vector2f(0.f, 0.f), target.normalized()), intersection, false))
      step.translation = intersection;
    else
      step.translation = target;
  }

  return theWalkGenerator.createPhase(step, lastPhase, 0.f);
}

void DribbleEngine::calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                             const Vector2f& ballInSCSNow, const Pose2f& scsCognition)
{
  if(std::abs(ballInSCSNow.angle() - ballSCS.angle()) > 45_deg)   // ball will land behind us, but is currently in front of us
  {
    const Angle velocityVector = scsCognition.rotation + motionRequest.ballEstimate.velocity.angle();
    const Vector2f minDistancePoint = ballInSCSNow.rotated(-velocityVector);
    const bool faraway = std::abs(minDistancePoint.x()) > minBallPositionFrontSide;
    const bool endPositionFar = ballSCS.squaredNorm() > sqr(minBallPositionFuture);
    if((faraway && endPositionFar))
    {
      const float distance = (ballSCS - ballInSCSNow).norm();
      ballSCS = ballInSCSNow + (ballSCS - ballInSCSNow).normalized(std::min(distance, minBallPositionFrontSide));
    }
  }
}
