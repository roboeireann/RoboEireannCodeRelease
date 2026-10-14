/**
 * @file REHysteresis.h
 *
 * Defines the HystThresholds and Comparator classes for working with 
 * hysteresis in a more consistent manner.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Streaming/AutoStreamable.h"

/**
 * define a class for specifying the high and low thresholds of a comparator
 * for hysteresis. This is useful for loading/defining parameters and is a
 * slightly more natural alternative than using Range with min and max
 *
 * @tparam T  the data type of the thresholds (and the input that will checked
 *            against them)
 */
template <typename T>
STREAMABLE(HystThresholds, 
{
  /**
   * Constructor.
   *
   * @param lower the lower threshold for hysteresis
   * @param upper the upper threshold for hysteresis
   */
  constexpr HystThresholds(T lower, T upper) : lower(lower) COMMA upper(upper){}

  Range<T> toRange() { return Range<T>(lower, upper); }

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
 * 
 * This functionality is used a lot in behaviour (and many places that a threshold
 * would be used) to avoid oscillation between two behaviours in the vicinity
 * of the threshold value
 *
 * The switching behaviour (with hysteresis) is as follows:
 * - the constructor specifies the initial output state of the comparator
 * - if the state is `stateWhenInputLow` when `update(...)` is called and the input 
 *   exceeds `upperThreshold` then the state switches to `stateWhenInputHigh`
 * - if the state is `stateWhenInputHigh` when `update(...)` is called and the input 
 *   dips below `lowerThreshold` then the state switches to `stateWhenInputLow`
 * 
 * Therefore to use it:
 * 1. initialise the comparator with the possible states and the initial state. 
 * 2. Set the thresholds either in the constructor or by subequently calling
 *    resetThresholds
 * 3. Call the `update(value)` function on each cycle/tick
 * 4. Check the `state` or `stateChanged` values sometime after the update and
 *    take whatever action is appropriate to the comparator output state or change
 *
 * @tparam Value  the type of the input value to the comparator
 * @tparam State  the type of the comparator output state (which is not always boolean)
 */
template <typename Value, typename State>
STREAMABLE(Comparator, 
{
  /**
   * Constructor to specify lower and upper threshold individually
   * 
   * @param lowerThreshold  lower threshold for hysteresis
   * @param upperThreshold  upper threshold for hysteresis
   * @param stateWhenInputLow   the comparator state when the input value dips below
   *                            the lower threshold
   * @param stateWhenInputHigh  the comparator state when the input value exceeds
   *                            the upper threshold
   * @param initialStateLow     if true the initial state is `stateWhenInputLow`
   *                            otherwise it is `stateWhenInputHigh`. The default
   *                            value is true.
   */
  constexpr Comparator(Value lowerThreshold, Value upperThreshold, State stateWhenInputLow, State stateWhenInputHigh,
                       bool initialStateLow = true)
      : lowerThreshold(lowerThreshold) COMMA upperThreshold(upperThreshold) COMMA stateWhenInputLow(stateWhenInputLow)
            COMMA stateWhenInputHigh(stateWhenInputHigh)
  {
    state = initialStateLow ? stateWhenInputLow : stateWhenInputHigh;
  }

  /**
   * Constructor to specify lower and upper threshold using HystThresholds
   * 
   * @param thresholds  a hysteresis object specifying the lower and upper thresholds
   * @param stateWhenInputLow   the comparator state when the input value dips below
   *                            the lower threshold
   * @param stateWhenInputHigh  the comparator state when the input value exceeds
   *                            the upper threshold
   * @param initialStateLow     if true the initial state is `stateWhenInputLow`
   *                            otherwise it is `stateWhenInputHigh`. The default
   *                            value is true.
   */
  constexpr Comparator(const HystThresholds<Value> &thresholds, State stateWhenInputLow, State stateWhenInputHigh,
                       bool initialStateLow = true)
      : lowerThreshold(thresholds.lower) COMMA upperThreshold(thresholds.upper)
            COMMA stateWhenInputLow(stateWhenInputLow) COMMA stateWhenInputHigh(stateWhenInputHigh)
  {
    state = initialStateLow ? stateWhenInputLow : stateWhenInputHigh;
  }

  /**
   * Constructor in which we don't specify lower and upper threshold yet, but will instead
   * specify them later in the `resetThresholds` function.
   * 
   * Specifying later in the `resetThresholds` funciton allows for configurable thresholds 
   * loaded from a file (after construction)
   * 
   * @param stateWhenInputLow   the comparator state when the input value dips below
   *                            the lower threshold
   * @param stateWhenInputHigh  the comparator state when the input value exceeds
   *                            the upper threshold
   * @param initialStateLow     if true the initial state is `stateWhenInputLow`
   *                            otherwise it is `stateWhenInputHigh`. The default
   *                            value is true.
   */
  constexpr Comparator(State stateWhenInputLow, State stateWhenInputHigh, bool initialStateLow = true)
      : lowerThreshold(Value()) COMMA upperThreshold(Value()) COMMA stateWhenInputLow(stateWhenInputLow)
            COMMA stateWhenInputHigh(stateWhenInputHigh)
  {
    state = initialStateLow ? stateWhenInputLow : stateWhenInputHigh;
  }

  /** 
   * reset the thresholds but doesn't change the comparator state
   * 
   * @param lowerThresholdIn  lower threshold for hysteresis
   * @param upperThresholdIn  upper threshold for hysteresis
   */
  void resetThresholds(Value lowerThresholdIn, Value upperThresholdIn)
  {
    lowerThreshold = lowerThresholdIn;
    upperThreshold = upperThresholdIn;
  }

  /**
   * reset the thresholds but doesn't change the comparator state
   *
   * @param thresholds  a hysteresis object specifying the lower and upper thresholds
   */ 
  void resetThresholds(const HystThresholds<Value> &thresholds)
  {
    lowerThreshold = thresholds.lower;
    upperThreshold = thresholds.upper;
  }

  /**
   * update the comparator with the most recent input (which may change its
   * state)
   * 
   * @param value  the input value to the comparator. This will be compared with
   *               the upper or lower threshold depending on the current
   *               comparator state
   */
  State update(Value value)
  {
    State prev = state;

    if ((state == stateWhenInputLow) && (value > upperThreshold))
      state = stateWhenInputHigh;
    else if ((state == stateWhenInputHigh) && (value < lowerThreshold))
      state = stateWhenInputLow;
    // otherwise don't change the state -- this is the hysteresis bit

    stateChanged = (state != prev);
    inputValue = value;

    return state;
  }

  Value lowerThreshold;     // not streamed
  Value upperThreshold;     // not streamed
  State stateWhenInputLow;  // not streamed
  State stateWhenInputHigh; // not streamed;

  /***** separate header from streamable members, note comma at end *****/, 
  (State) state,
  (bool) stateChanged, // true after an update that changed state - useful for edge triggering
  (Value) inputValue,  // the most recent input value that produced the current state
});

using Comparatora = Comparator<Angle, bool>;
using Comparatori = Comparator<int, bool>;
using Comparatoru = Comparator<unsigned, bool>;
using Comparatorf = Comparator<float, bool>;
