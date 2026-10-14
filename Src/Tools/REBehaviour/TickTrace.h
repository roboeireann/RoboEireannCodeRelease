/**
 * @file TickTrace.h
 *
 * A trace of (Coro and HSM) tasks ticked during a behaviour tick in the order
 * that they were ticked
 *
 * Inspired by cabsl::ActivationGraph by Thomas Röfer (B-Human).
 * See Acknowledgements.md for details.
 */

#pragma once

#include "HbsTypes.h"

#include <string>
#include <vector>

namespace Hbs
{

/**
 * Trace info for a single task ticked during a behaviour tick.
 */
struct TaskFrame
{
  int depth = 0;                   ///< call nesting depth (0 = top-level)
  const char* taskName = "";       ///< task method name
  const char* stateName = "";      ///< coro / HSM state name at end of tick
  Time taskDuration = 0;           ///< time since task last reset
  Time stateDuration = 0;          ///< time in current state
  unsigned taskTicks = 0;          ///< number of ticks since last reset
  unsigned stateTicks = 0;         ///< number of ticks in current state
  std::vector<std::string> contextStrings; ///< optional per-tick annotations providing context that led to a particular state (e.g. args)
  std::vector<std::string> strings; ///< optional per-tick annotations (usually during state execution)
  Status status = Status::INITIAL;

  void addContextString(std::string s) { strings.push_back(std::move(s)); }
  void addString(std::string s) { strings.push_back(std::move(s)); }
};


/**
 * A complete trace of all tasks ticked during one behaviour tick, in tick order.
 *
 * Typical usage:
 *
 *   trace.beginTick(); // at the start of the overall behaviour tick
 *
 *   // on each task entry
 *   int frameIdx = trace.beginTaskFrame(taskName, taskDuration);
 *
 *   // ... task body executes ...
 *
 *   // on task exit
 *   trace.endTaskFrame(frameIdx, status, stateName, stateDuration);
 */
class TickTrace
{
public:
  TickTrace() { frames.reserve(32); }

  /// Call once before ticking any tasks each behaviour cycle.
  void beginTick()
  {
    frames.clear();
    currentDepth = 0;
  }

  /// Record a task starting its tick. Returns the frame index for use in endTaskFrame.
  int beginTaskFrame(const char* taskName, Time taskDuration, unsigned taskTicks)
  {
    int idx = static_cast<int>(frames.size());
    TaskFrame& f = frames.emplace_back();
    f.depth        = currentDepth;
    f.taskName     = taskName;
    f.taskDuration = taskDuration;
    f.taskTicks    = taskTicks;
    ++currentDepth;
    return idx;
  }

  /// Complete a task's frame with its final status and state info (known after body executes).
  void endTaskFrame(int frameIdx, Status status, const char* stateName, Time stateDuration, unsigned stateTicks)
  {
    --currentDepth;
    frames[frameIdx].status        = status;
    frames[frameIdx].stateName     = stateName;
    frames[frameIdx].stateDuration = stateDuration;
    frames[frameIdx].stateTicks    = stateTicks;
  }


  /// add a context string to the extra info associated with a task frame
  void addContextFrameString(int frameIdx, std::string s)
  {
    if (0 <= frameIdx && frameIdx < static_cast<int>(frames.size()))
        frames[frameIdx].addContextString(std::move(s));
  }
  
  /// add a string to the extra info associated with a task frame
  void addFrameString(int frameIdx, std::string s)
  {
    if (0 <= frameIdx && frameIdx < static_cast<int>(frames.size()))
        frames[frameIdx].addString(std::move(s));
  }

  const std::vector<TaskFrame>& getFrames() const { return frames; }  

private:
  int currentDepth = 0;
  std::vector<TaskFrame> frames;
};

} // namespace Hbs
