/**
 * @file FrameInfoEx.h
 * Extended information about the current frame
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct FrameInfoEx
 * A struct that contains extended information on the current frame.
 */
STREAMABLE_WITH_BASE(FrameInfoEx, FrameInfo,
{
  /**
   * The method returns the time difference between a given timestamp and the
   * current frame time.
   * @param timestamp A timestamp, usually in the past.
   * @return The number of ms passed since the given timestamp.
   */
  int getTimeSincePrevious(unsigned timestamp) const;

  /** Draw this representation. */
  void draw() const;  

  /**** separate header from streamable fields - comma at end of comment ****/,

  (unsigned)(0) frameNumber, ///< a simple counter incremented on each frame
  (unsigned)(0) prevFrameTime, ///< The timestamp of the previous frame in ms
  (unsigned)(0) frameInterval, ///< the interval between frames - always zero for frame 1
});

inline int FrameInfoEx::getTimeSincePrevious(unsigned timestamp) const
{
  return static_cast<int>(prevFrameTime - timestamp);
}

//STREAMABLE_WITH_BASE(CognitionFrameInfo, FrameInfo, {,});
