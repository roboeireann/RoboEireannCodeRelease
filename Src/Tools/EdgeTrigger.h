/**
 * @file EdgeTrigger.h
 *
 * Defines classes for edge triggering when an input changes
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"


/**
 * define a generic edge trigger class that eliminated repeated code and variables
 * used just for edge triggering
 */
template<typename Value>
STREAMABLE(EdgeTrigger,
{
  /**
   * Constructor.
   * Specify lower and upper threshold individually
   */
  constexpr EdgeTrigger(Value initial) : value(initial) COMMA prevValue(initial)
  {
  }

  /**
   * Use this constructor if you want the initial configuration to be as if the value just changed
   */
  constexpr EdgeTrigger(Value initial, Value prevValue) : value(initial) COMMA prevValue(prevValue)
  {
  }

  void reset(Value initial) { reset(initial, initial); }
  void reset(Value initial, Value prev)
  {
    value = initial;
    prevValue = prev;
  }

  /**
   * true if the value changed at the last update
   */
  bool valueChanged() { return value != prevValue; }

  /**
   * returns true if the value changed (i.e. if we should edge trigger)
   */
  bool update(Value newValue)
  {
    prevValue = value;
    value = newValue;

    return value != prevValue;
  }
  

  /***** separate header from streamable members, note comma at end of line *****/,

  (Value) value, // most recently updated value
  (Value) prevValue, // the value before the most recent update
});


using EdgeTriggera = EdgeTrigger<Angle>;
using EdgeTriggeri = EdgeTrigger<int>;
using EdgeTriggeru = EdgeTrigger<unsigned>;
using EdgeTriggerf = EdgeTrigger<float>;
