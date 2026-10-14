/**
 * @file KeyStates.h
 *
 * Declaration of struct KeyStates
 *
 * @author unknown
 * @author Jesse Richter-Klug
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "Streaming/Function.h"

/**
 * The struct represents the states of the keys.
 */
STREAMABLE(KeyStates,
{
  ENUM(Key,
  {,
    // Nao touch sensors:
    nao_firstKey,
    headFront = nao_firstKey,
    headMiddle,
    headRear,
    lHandBack,
    lHandLeft,
    lHandRight,
    rHandBack,
    rHandLeft,
    rHandRight,

    // Nao bumpers:
    lFootLeft,
    lFootRight,
    rFootLeft,
    rFootRight,
    chest,

    nao_lastKey = chest,

    // BoosterK1
    k1_firstKey,
    k1_f1 = k1_firstKey,
    k1_stand,
    k1_walk,
    k1_lastKey = k1_walk
  });

  KeyStates() { pressed.fill(false); }

  /**** separate header from streamable fields, note comma after comment ****/,

  (ENUM_INDEXED_ARRAY(bool, Key)) pressed,
});


STREAMABLE_WITH_BASE(EnhancedKeyStates, KeyStates,
{
  EnhancedKeyStates() { hitStreak.fill(0); pressedDuration.fill(0); }
  
  /*
   * calculates the 'current' hit streak for a given key state sensor
   *
   * @param key the sensor
   * @param maxPressDuration is if the sensor is triggered longer than this, the hit streak ends
   * @param allowedTimeBetweenHits max allowed time between to hits to be considered part of a hit streak
   *
   * @return the size of the current finished hit streak recently
   */
  FUNCTION(unsigned(KeyStates::Key key, unsigned maxPressDuration, unsigned allowedTimeBetweenHits)) getHitStreakFor;

  /*
   * checks whether a button press just reached a certain duration in this frame
   *
   * @param key the sensor
   * @param duration the duration for which to check
   *
   * @return whether a button press just reached a certain duration in this frame
   */
  FUNCTION(bool(KeyStates::Key key, unsigned duration)) isPressedFor;

  /**** separate header from streamable fields - comma at end of comment ****/,

  (ENUM_INDEXED_ARRAY(unsigned, Key)) hitStreak,       ///< hit streak per key calculated with default parameters of 
                                                       ///< the provider. Valid for just one tick, shortly after key last released.
  (ENUM_INDEXED_ARRAY(unsigned, Key)) pressedDuration, ///< duration per key while the key is pressed.
                                                       ///< Zero if the key is just pressed this frame, or has been released.
  (ENUM_INDEXED_ARRAY(unsigned, Key)) clickedDuration, ///< duration of a click (release) just detected for key, values for each key.
                                                       ///< Non zero in the cycle where a key release is detected. Zero at all other times.
});
