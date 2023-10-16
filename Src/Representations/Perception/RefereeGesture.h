/**
 * @file RefereeGesture.h
 *
 * A representation for the perceived referee gesture
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(RefereeGesture,
{
  ENUM(Gesture,
  {,
    NONE, // no gesture present/recognised
    KICK_IN,
    GOAL_KICK,
    CORNER_KICK,
    GOAL,
    PUSHING_FREE_KICK,
    FULL_TIME,
  });

  ENUM(Team,
  {,
    BLUE_TEAM,
    RED_TEAM,
    NO_TEAM,
  })
  ,

  (Gesture)(NONE) gesture,  ///< the most recently detected gesture
  (Team)(NO_TEAM) team,     ///< which team the gesture referred to
  (unsigned int)(0) lastDetectionTime,  ///< the time at which the gesture was detected
});