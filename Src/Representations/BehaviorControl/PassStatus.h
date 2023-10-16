/**
 * @file Representations/BehaviorControl/PassStatus.h
 *
 * Representation of the passes by the team
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <limits>

/**
 * @struct TimeToReachBall
 * Representation of the Time to reach the ball
 */
STREAMABLE(PassStatus,
{,
  (int)(-1) passKicker, ///< player who intends to pass (-1 for none)
  (int)(-1) passReceiver, ///< player who should receive the intended pass (-1 for none)
  // (int)(-1) prevPassReceiver, ///< the player number of the previous pass target (-1 for none)
  (int)(0) numPasses, ///< substantial pass attempts (ball travelled substantially)
});
