/**
 * @file Representations/BehaviorControl/TimeToReachBall.h
 *
 * Representation of the estimated time to reach the ball
 *
 * @author Andreas Stolpmann
 * @note BH2021 Code release - only required for RE2025 and older behaviours
 */

#pragma once

#include "Streaming/AutoStreamable.h"
#include <limits>

/**
 * @struct TimeToReachBall
 * Representation of the Time to reach the ball
 */
STREAMABLE(TimeToReachBall,
{,
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBall,         /**< The estimated time when reach the ball */
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBallStriker,
});
