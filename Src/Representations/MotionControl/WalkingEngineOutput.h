/**
 * @file WalkingEngineOutput.h
 *
 * This file declares a representation that contains the static output of the walking engine.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"


STREAMABLE(BrakingModelParameters,
{,    // d[m] = (quadA * v^2) + (quadB * v) + quadC
  (float)(0.f) quadA, /**< Quadratic coefficient (s^2/m). */
  (float)(0.f) quadB, /**< Linear coefficient (s). */
  (float)(0.f) quadC, /**< Constant term (m); accounts for the zero-speed brake distance floor. */
  (float)(1.f) distanceSafetyFactor, /**< Scales available braking distance; 1.0 = use model as-is, 0.5 = assume only half the actual distance is usable. */
  (float)(0.f) minCappedSpeed, /**< Commanded body-speed magnitude [m/s] below which the cap is skipped (fine-positioning escape). */
});

STREAMABLE(WalkingEngineOutput,
{,
  (Pose2f)(45_deg, 100.f, 100.f) maxSpeed, /**< The maximum speed possible. */
  (float)(100.f) maxSpeedBackwards, /**< The maximum backward speed possible. */
  (float)(0.25f) walkStepDuration, /**< Duration of one step in seconds. */

  (Pose2f)(45_deg, 50.f, 50.f) maxStepSize, /**< The maximum step size. */
  (float)(30.f) maxBackwardStepSize, /**< The maximum backward speed possible. */
  (float)(0.25f) stepDuration, /**< Step duration (in s) */

  (Pose2f)(45_deg, 50.f, 50.f) maxPossibleStepSize, /**< The maximum step size that is theoretical possible. DO NOT use for walking. */
  (float)(30.f) maxPossibleBackwardStepSize, /**< The maximum backward speed that is theoretical possible. DO NOT use for walking. */

  (Pose2f)(45_deg, 50.f, 50.f) energyEfficientWalkSpeed, /**< In this walk speed the robot can walk higher and the knees heat up slower. */
  (Pose2f)(45_deg, 50.f, 50.f) noEfficientWalkSpeed, /**< In this walk speed the robot can only walk at the normal height. */

  (BrakingModelParameters) brakingModel, /**< Empirical braking-distance model used to cap approach speed by distance to target. */
});
