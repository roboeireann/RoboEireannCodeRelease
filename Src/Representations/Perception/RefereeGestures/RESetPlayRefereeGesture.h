/**
 * @file RESetPlayRefereeGesture.h
 *
 * A representation for the perceived referee gestures related to set plays
 *
 * @author Rudi Villing
 * @author Conal Hughes
 */

#pragma once

#include "Streaming/AutoStreamable.h"

STREAMABLE(RESetPlayRefereeGesture,
{
  ENUM(Gesture,
  {,
    NONE, // no gesture present/recognised
    TOWARDS_LEFT, // referee arm is pointing towards left side of the field (from perspective of game controller operator)
    TOWARDS_RIGHT, // referee arm is pointing towards right side of the field (from perspective of game controller operator)
  });

  /******** streamable members follow (note comma at end of comment) ********/,

  // (bool)(false)     isNewResult, ///< indicates if this is new result in the current frame or a repeated old result (e.g. due to inference in progress)
  (Gesture)(NONE)   detectedGesture, ///< indicates which Gesture (if any) was detected in this frame
  (float)(0.f)      confidence, ///< the confidence value of gesture being valid
  (Gesture)(NONE)   lastDetection, ///< indicates the last gesture detected that was not NONE (only NONE before first detection)
  (unsigned)(0)     frameTimeLastDetection, ///< the frame time of the last detection (0 if no last detection)
});