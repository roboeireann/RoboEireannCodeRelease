/**
 * @file: TimingStats.h
 *
 * Measure timing and accumulate. Provide avg/min/max stats and reset on demand.
 * This does not implement a rolling window, but a hard reset at the end of the
 * interval (to avoid a ringbuffer).
 *
 * @author Rudi Villing
 */

#pragma once

#include "Platform/Time.h"
#include "FmtFormatter.h"

class IntervalTimingStats
{
public:
  using Interval = double;

  IntervalTimingStats() { reset(); }

  void reset()
  {
    startTime = 0;
    currentTiming = 0;
    sumTimings = 0;
    nTimings = 0;
    minTiming = 60 * 60 * 1000; // 1hr is ms
    maxTiming = 0;
  }

  // to start a new interval or resume after a pause, just use start()
  void start() { startTime = Time::getCurrentSystemTime(); }
  void pause() { currentTiming += (Time::getCurrentSystemTime() - startTime); startTime = 0; }
  void stop() 
  {
    if (startTime)
    {
      currentTiming += (Time::getCurrentSystemTime() - startTime);
      startTime = 0;
    }
    if (currentTiming) // might be zero, e.g. first time we call stopStart
    {
      sumTimings += currentTiming;
      ++nTimings;

      if (currentTiming > maxTiming)
        maxTiming = currentTiming;
      if (currentTiming < minTiming)
        minTiming = currentTiming;

      currentTiming = 0;
    }
  }
  void stopStart() { stop(); start(); }

  unsigned min() const { return minTiming; }
  unsigned max() const { return maxTiming; }
  unsigned sum() const { return sumTimings; }
  Interval avg() const { return nTimings ? static_cast<Interval>(sumTimings) / nTimings : sumTimings; }
  unsigned n() const { return nTimings; }

private:
  unsigned startTime;
  unsigned currentTiming;
  unsigned sumTimings; // ms won't roll over in the time we care about
  unsigned nTimings;
  unsigned minTiming;
  unsigned maxTiming;
};

FMT_FORMATTER(IntervalTimingStats, s, "avg/min/max (ms) = {:.1f}/{}/{} (N={})", s.avg(), s.min(), s.max(), s.n());
