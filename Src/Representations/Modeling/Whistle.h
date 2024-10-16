/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Whistle,
{,
  (float)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability. */
  (unsigned char)(0) channelsUsedForWhistleDetection,  /**< Number of channels the robot used to listen. */
  (unsigned int)(0)  lastTimeWhistleDetected,          /**< Timestamp */
  (bool)(false)      whistleDetected,                  /**< Whether the whistle was detected. */
});
