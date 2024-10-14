/**
 * @file Hysteresis.h
 *
 * Defines classes for working with hysteresis
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"


/**
 * define a class for specifying the high and low thresholds of a comparator
 * for hysteresis. This is useful for loading/defining parameters and is a
 * slightly more natural alternative than using Range with min and max
 */
template<typename T>
STREAMABLE(HystThresholds,
{
  /**
   * Constructor.
   */
  constexpr HystThresholds(T lower, T upper) : lower(lower) COMMA upper(upper)
  {
  }

  /***** separate header from streamable members, note comma at end *****/,
  (T) lower,
  (T) upper,
});

class Angle;

using HystThresholdsa = HystThresholds<Angle>;
using HystThresholdsi = HystThresholds<int>;
using HystThresholdsu = HystThresholds<unsigned>;
using HystThresholdsf = HystThresholds<float>;


/**
 * define a generic comparator class that can have one of two possible outputs
 * depending on the input value and the most recent output value
 */
template<typename Value, typename State>
STREAMABLE(Comparator,
{
  /**
   * Constructor.
   * Specify lower and upper threshold individually
   */
  constexpr Comparator(Value lowerThreshold, Value upperThreshold, State stateLow, State stateHigh,
                       bool initialStateLow = true)
      : lowerThreshold(lowerThreshold) COMMA upperThreshold(upperThreshold) COMMA stateLow(stateLow)
            COMMA stateHigh(stateHigh)
  {
    state = initialStateLow ? stateLow : stateHigh;
  }

  /**
   * Constructor.
   * Specify lower and upper threshold using HystThresholds
   */
  constexpr Comparator(const HystThresholds<Value>& thresholds, State stateLow, State stateHigh, bool initialStateLow = true)
      : lowerThreshold(thresholds.lower) COMMA upperThreshold(thresholds.upper) COMMA stateLow(stateLow)
            COMMA stateHigh(stateHigh)
  {
    state = initialStateLow ? stateLow : stateHigh;
  }

  /**
   * Constructor.
   * Don't specify lower and upper threshold yet - it will have to be done later in the resetThresholds function
   * This allows for configurable thresholds loaded from a file (after construction)
   */
  constexpr Comparator(State stateLow, State stateHigh, bool initialStateLow = true)
      : lowerThreshold(Value()) COMMA upperThreshold(Value()) COMMA stateLow(stateLow) COMMA stateHigh(stateHigh)
  {
    state = initialStateLow ? stateLow : stateHigh;
  }

  /** reset the thresholds but doesn't change the state - only update changes state  */
  void resetThresholds(Value lowerThresholdIn, Value upperThresholdIn)
  {
    lowerThreshold = lowerThresholdIn;
    upperThreshold = upperThresholdIn;
  }

  /** reset the thresholds but doesn't change the state - only update changes state  */
  void resetThresholds(const HystThresholds<Value>& thresholds)
  {
    lowerThreshold = thresholds.lower;
    upperThreshold = thresholds.upper;
  }


  State update(Value value)
  {
    State prev = state;

    if ((state == stateLow) && (value > upperThreshold))
      state = stateHigh;
    else if ((state == stateHigh) && (value < lowerThreshold))
      state = stateLow;
    // otherwise don't change the state -- this is the hysteresis bit

    stateChanged = (state != prev);
    inputValue = value;

    return state;
  }

  

  Value lowerThreshold; // not streamed
  Value upperThreshold; // not streamed
  State stateLow; // not streamed
  State stateHigh; // not streamed;

  /***** separate header from streamable members, note comma at end *****/,
  (State) state,
  (bool) stateChanged, // true after an update that changed state - useful for edge triggering
  (Value) inputValue, // the most recent input value that produced the current state
});


using Comparatora = Comparator<Angle, bool>;
using Comparatori = Comparator<int, bool>;
using Comparatoru = Comparator<unsigned, bool>;
using Comparatorf = Comparator<float, bool>;
