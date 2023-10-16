/**
 * @file RefereeGestureDetector.h
 *
 * Module that detects referee gestures from the camera image
 *
 * @author Rudi Villing
 */

#pragma once

#include <array>

namespace DetectedGesture
{
  enum Gesture
  {
    CORNER_BLUE,
    CORNER_RED,
    FULL_TIME,
    GOAL_KICK_BLUE,
    GOAL_KICK_RED,
    GOAL_BLUE,
    GOAL_RED,
    KICK_IN_BLUE,
    KICK_IN_RED,
    PUSHING_BLUE,
    PUSHING_RED,
    UNRECOGNISED,
    INFERENCE_NOT_READY
  };

  typedef std::array<int,12> GestureOneHot;
};