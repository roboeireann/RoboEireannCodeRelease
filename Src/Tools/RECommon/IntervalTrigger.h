/**
 * @file IntervalTrigger.h
 *
 * Utility classes for rate-limiting actions, either by elapsed time or tick count.
 * Intended for use as member variables in classes that need periodic triggereding.
 *
 * Usage:
 *   IntervalTriggerMs logIntervalTrigger{500};
 *   if (logIntervalTrigger.updateTriggered(nowMs)) doSomething();
 *
 *   IntervalTriggerN sampleIntervalTrigger{10};
 *   if (sampleIntervalTrigger.updateTriggered()) doSomethingElse();
 *
 * @author Rudi Villing
 */

#pragma once

/**
 * Triggers periodically based on elapsed time in milliseconds.
 * Handles timer wraparound correctly.
 */
struct IntervalTriggerMs
{
  IntervalTriggerMs() : initialIntervalMs(15000u), dueMs(0) {}
  explicit IntervalTriggerMs(unsigned intervalMs) : initialIntervalMs(intervalMs), dueMs(0) {}

  bool updateTriggered(unsigned nowMs) { return updateTriggered(nowMs, initialIntervalMs); }
  bool updateTriggered(unsigned nowMs, unsigned intervalMs)
  {
    if ((dueMs == nowMs) || ((dueMs - nowMs) > intervalMs)) // handles wrap around
    {
      // try to avoid accumulating timing drift if possible.
      // auto reset the interval start if we are more than half an interval overdue
      // (likely due to not executing this triggered for some cycles)
      unsigned base = (dueMs == 0 || (nowMs - dueMs) > intervalMs / 2) ? nowMs : dueMs;
      dueMs = base + intervalMs;
      triggeredLastUpdate = true;
    }
    else
      triggeredLastUpdate = false;

    return triggeredLastUpdate;
  }

  bool wasTriggered() const { return triggeredLastUpdate; }

  void reset() { dueMs=0; }

private:
  unsigned initialIntervalMs;
  unsigned dueMs;
  bool triggeredLastUpdate = false;
};

/**
 * Triggers every N calls to updateTriggered().
 * Fires on the Nth tick, then resets.
 */
struct IntervalTriggerN
{
  IntervalTriggerN() : initialInterval(1000u), count(0) {}
  explicit IntervalTriggerN(unsigned initialInterval) : initialInterval(initialInterval), count(0) {}

  bool updateTriggered() { return updateTriggered(initialInterval); }
  bool updateTriggered(unsigned interval)
  {
    if (++count >= interval)
    {
      count = 0;
      triggeredLastUpdate = true;
    }
    else
      triggeredLastUpdate = false;

    return triggeredLastUpdate;
  }

  bool wasTriggered() const { return triggeredLastUpdate; }

  void reset() { count = 0; }

private:
  unsigned initialInterval;
  unsigned count;
  bool triggeredLastUpdate = false;
};