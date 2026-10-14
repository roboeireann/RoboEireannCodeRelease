/**
 * @file KickAccuracyEvaluation.h
 *
 * Per-kick accuracy measurements (commanded vs. actual ball trajectory),
 * computed from simulation ground truth by the KickAccuracyProvider.
 * Only available in simulation scenes that activate the provider.
 *
 * @author James Petri
 */

#pragma once

#include "Math/Angle.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE(KickAccuracyEvaluation,
{,
  (unsigned)(0) numKicksMeasured,
  (Angle)(0_deg) lastCommandedDirection,  /**< World frame. */
  (Angle)(0_deg) lastActualDirection,     /**< World frame. */
  (Angle)(0_deg) lastDirectionError,      /**< actual - commanded. */
  (float)(0.f) lastCommandedLength,       /**< mm */
  (float)(0.f) lastAchievedDistance,      /**< mm */
  (bool)(false) lastMeasurementTainted,   /**< Ball was likely touched before it stopped. */
  (Angle)(0_deg) meanDirectionError,      /**< Running mean over untainted kicks. */
  (Angle)(0_deg) meanAbsDirectionError,   /**< Running mean of |dirError|, untainted kicks - scatter measure. */
  (float)(0.f) meanDistanceError,         /**< Running mean of achieved - commanded (mm), untainted kicks. */
});
