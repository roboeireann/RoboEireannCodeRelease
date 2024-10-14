/**
 * @file RefereeGesture.h
 *
 * A representation for the perceived READY gesture in the STANDBY state
 *
 * @author James Petri
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(StandbyReadyGesture,
{,
  (bool)(false)     isDetected, ///< states if the Gesture was detected in this frame
  (float)(0.f)      confidence, ///< the confidence value of gesture being valid
  (unsigned)(0)     frameTimeLastDetection, ///< the frame time at which the gesture was last detected (0 for never)
});