/**
 * @file DribbleEngine.h
 *
 * This file declares a module that provides a dribble generator.
 *
 * @author Philip Reichenberg
 * @author Arne Hasselbring
 */

#pragma once

#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/DribbleGenerator.h"
#include "Representations/MotionControl/KickGenerator.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkGenerator.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/WalkKickGenerator.h"
#include "Representations/MotionControl/WalkStepData.h"
#include "Representations/MotionControl/WalkToBallGenerator.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Framework/Module.h"

MODULE(DribbleEngine,
{,
  REQUIRES(BallSpecification),
  REQUIRES(FrameInfo),
  REQUIRES(KickGenerator),
  REQUIRES(KickInfo),
  USES(MotionInfo),
  REQUIRES(OdometryDataPreview),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  REQUIRES(WalkGenerator),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(WalkKickGenerator),
  REQUIRES(WalkStepData),
  REQUIRES(WalkToBallGenerator),
  PROVIDES(DribbleGenerator),
  DEFINES_PARAMETERS(
  {,
    (Angle)(3_deg) redecideSignThreshold, /**< The dribble foot is only re-decided if the decision angle is larger than this. */
    (float)(100.f) minBallPositionFuture, /**< Ball must land this far away from us if it is rolling towards us. */
    (float)(500.f) minBallPositionFrontSide, /**< Ball must be this far away relative to the closest point it will have to us. */
    (float)(300.f) minBallVelocityCloseRange, /**< Clip ball velocity to this value when close to the ball. */
    (Rangef)(250.f, 2000.f) ballVelocityInterpolationRange, /**< Based on the current ball distance interpolate the velocity. */
    (float)(800.f) maxBallVelocity, /**< Max ball velocity to still allow a kick. */
    (float)(100.f) k1KickPositionTolerance, /**< [K1] Ball-to-expected-position tolerance (mm) for triggering the RL kick. */
    (Angle)(6_deg) k1KickDirectionTolerance, /**< [K1] Target-direction tolerance for triggering the RL kick, used when the behavior provides no wider precision range. */
    (Angle)(15_deg) k1KickDirectionPrecisionCap, /**< [K1] Upper cap on widening the direction gate from motionRequest.directionPrecision. */
    (bool)(true) walkThroughBallInsteadOfKick, /**< Fallback: do not kick at all, just walk through the ball aligned to the dribble direction. */
    (float)(100.f) walkThroughBallReach, /**< [walk-through] Aim this far (mm) past the ball along the dribble direction, so the robot keeps pushing through it. */
  }),
});

class DribbleEngine : public DribbleEngineBase
{
  void update(DribbleGenerator& dribbleGenerator) override;

  /**
   * Calculates the pose where the robot should walk to be able to dribble the ball with one step.
   * @param ballInSCS The ball position relative to the new support foot frame.
   * @param directionInSCS The direction relative to the new support foot frame to which the ball should be dribbled.
   * @param sign The foot sign with which to dribble (updated in \c lastSign before calling, see \c updateDribbleSign).
   * @param turnKickAllowed Is the turn kick interpolation allowed?
   * @return The pose from which the ball can be dribbled with one step.
   */
  Pose2f calcBasePose(const Vector2f& ballInSCS, Angle directionInSCS, float sign, const bool turnKickAllowed) const;

  /**
   * Updates \c lastSign, the hysteresis decision with which foot to dribble.
   * Both the kick-trigger check and the pose the robot walks to must use this same sign,
   * so they always target the same foot pocket.
   * @param ballInSCS The ball position relative to the new support foot frame.
   * @param directionInSCS The direction relative to the new support foot frame to which the ball should be dribbled.
   */
  void updateDribbleSign(const Vector2f& ballInSCS, Angle directionInSCS);

  /**
   * Creates a walk phase that pushes the robot straight through the ball, aligned to the dribble
   * direction, at the requested walk speed. Used as a no-kick fallback (walkThroughBallInsteadOfKick).
   * @param motionRequest The current motion request (for the walk speed ratio).
   * @param ballInSCS The ball position relative to the new support foot frame.
   * @param directionInSCS The dribble direction relative to the new support foot frame.
   * @param isLeftPhase Whether the next phase is a left phase.
   * @param lastPhase The previous MotionPhase.
   * @return A walk phase heading through the ball.
   */
  std::unique_ptr<MotionPhase> createWalkThroughBallPhase(const MotionRequest& motionRequest, const Vector2f& ballInSCS,
                                                          const Angle directionInSCS, const bool isLeftPhase,
                                                          const MotionPhase& lastPhase) const;

  float lastSign = 1.f; /**< The foot sign of the last phase generated by this engine. */

  /**
   * Calculate a temp kick pose in case the ball is rolling towards us.
   * We can not trust the perceived ball velocity and direction. Therefore clip the ball position, to avoid turning away from the ball.
   * @param motionRequest Motion request to access original ball velocity
   * @param ballSCS ball position used to determine the kick pose
   * @param ballInSCSNow current used relative ball position
   * @param scsCognition transformation pose to get a relative field position into a zero-step relative position (only rotation is used)
   */
  void calcInterceptionPosition(const MotionRequest& motionRequest, Vector2f& ballSCS,
                                const Vector2f& ballInSCSNow, const Pose2f& scsCognition);
};
