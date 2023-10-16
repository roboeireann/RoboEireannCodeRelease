/**
 * @file WhistleProcessed.h
 *
 * whistle data as processed by the whistle handler in a form that can be used
 * elsewhere outside the normal RoboCup state machine
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(WhistleProcessed,
{,
  (bool)(false)  thisRobotDetected,            ///< detected by this robot in this cycle
  (unsigned)(0)  thisRobotLastDetectionStartTime,   ///< Timestamp of the start of the most recent detection

  (int)(0)       numRobotsHeardSomething,      ///< this robot + teammates - heard something is not the same as detecting the whistle with sufficient confidence

  (bool)(false)  detected,                     ///< overall detection status for whistle (this robot and teammates) in this cycle
  (unsigned)(0)  lastDetectionStartTime,            ///< timestamp of the start of the most recent detection
});
