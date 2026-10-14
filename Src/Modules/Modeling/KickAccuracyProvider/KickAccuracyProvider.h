/**
 * @file KickAccuracyProvider.h
 *
 * Sim-only tuning aid: measures, for every executed kick, the commanded
 * direction/length (from the MotionRequest, in the ground-truth world frame)
 * against the actual ball trajectory (from GroundTruthWorldState) and reports
 * the direction and distance error per kick plus a running mean.
 *
 * Activated per scene via module request, e.g. in BoosterFast.con:
 *   for Cognition mr KickAccuracyEvaluation KickAccuracyProvider
 * It REQUIRES GroundTruthWorldState, so it cannot run on a real robot.

 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"
#include "Representations/Infrastructure/KickAccuracyEvaluation.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"

MODULE(KickAccuracyProvider,
{,
  REQUIRES(FallDownState),
  REQUIRES(FrameInfo),
  REQUIRES(GroundTruthWorldState),
  REQUIRES(MotionInfo),
  REQUIRES(MotionRequest),
  PROVIDES(KickAccuracyEvaluation),
  DEFINES_PARAMETERS(
  {,
    (float)(100.f) minBallMoveDistance, /**< Ball displacement (mm) from kick start that counts as contact. */
    (float)(50.f) stopSpeedThreshold,   /**< Below this ball speed (mm/s) the ball counts as stopped. */
    (int)(200) stopHoldTime,            /**< The ball must stay slow this long (ms) before finalizing. */
    (int)(3000) maxWaitForContact,      /**< Declare a whiff this long (ms) after kick start without contact. */
    (int)(15000) maxTrackTime,          /**< Finalize anyway this long (ms) after kick start. */
    (float)(300.f) taintSpeedJump,      /**< Speed increase (mm/s) while rolling counts as a touch by someone else. */
    (float)(200.f) headingMinSpeed,     /**< Only evaluate the ball's velocity heading above this speed (mm/s). */
    (Angle)(20_deg) deflectionAngle,    /**< Heading deviation from the initial roll direction that counts as a deflection. */
    (float)(5000.f) maxCommandLength,   /**< Commanded lengths are clamped to this for error stats; matches the policy's maxKickLength normalization cap (goal shots command FLT_MAX). */
  }),
});

class KickAccuracyProvider : public KickAccuracyProviderBase
{
  void update(KickAccuracyEvaluation& kickAccuracyEvaluation) override;

  void finalize(const Vector2f& ballRest, KickAccuracyEvaluation& kickAccuracyEvaluation);

  enum State
  {
    idle,     /**< Waiting for a kick phase to start. */
    kicking,  /**< Kick phase started, waiting for ball contact. */
    tracking, /**< Ball is moving, waiting for it to come to rest. */
  };

  State state = idle;
  MotionPhase::Type lastExecutedPhase = MotionPhase::playDead;

  // Commanded kick, cached while the behavior requests it (world frame).
  Angle cachedWorldDirection = 0_deg;
  float cachedLength = 0.f;

  // Snapshot at kick phase start.
  Angle commandedDirection = 0_deg;
  float commandedLength = 0.f;
  Vector2f ballStart = Vector2f::Zero();
  unsigned kickStartTime = 0;

  bool tainted = false;
  bool fell = false; /**< The robot fell during the kick or while the ball was rolling. */
  float lastSpeed = 0.f;
  unsigned slowSince = 0; /**< Time the ball first dropped below stopSpeedThreshold; 0 = not slow. */
  Angle referenceHeading = 0_deg; /**< Roll direction shortly after contact, for deflection detection. */
  bool hasReferenceHeading = false;

  unsigned numUntainted = 0;
  float directionErrorSum = 0.f;    /**< rad, over untainted kicks. */
  float absDirectionErrorSum = 0.f; /**< rad, over untainted kicks. */
  float distanceErrorSum = 0.f;     /**< mm, over untainted kicks. */
};
