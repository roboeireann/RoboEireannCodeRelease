/**
 * @file Threads/Cognition.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 */

#include "Modules/Infrastructure/InterThreadProviders/PerceptionProviders.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"

#include "Representations/Communication/TeamMessage2024.h"

#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "Tools/Settings.h"

#include "Tools/TextLogging.h"

#include <cstring>

REGISTER_EXECUTION_UNIT(Cognition)

thread_local bool Cognition::isUpper = false;

DECL_TLOGGER(tlogger, "Cognition", TextLogging::DEBUG);

Cognition::Cognition()
{
  Blackboard::getInstance().alloc<UpperFrameInfo>("UpperFrameInfo").time = 100000;
  Blackboard::getInstance().alloc<LowerFrameInfo>("LowerFrameInfo").time = 100000;

  InMapFile stream("cognition.cfg");
  ASSERT(stream.exists());
  stream >> params;
}

Cognition::~Cognition()
{
  Blackboard::getInstance().free("UpperFrameInfo");
  Blackboard::getInstance().free("LowerFrameInfo");
}

bool Cognition::beforeFrame()
{
  const FrameInfo* lowerFrameInfo = Blackboard::getInstance().exists("LowerFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["LowerFrameInfo"]))
                                    : nullptr;
  const FrameInfo* upperFrameInfo = Blackboard::getInstance().exists("UpperFrameInfo")
                                    ? static_cast<FrameInfo*>(const_cast<Streamable*>(&Blackboard::getInstance()["UpperFrameInfo"]))
                                    : nullptr;
  unsigned lowerFrameTime = lowerFrameInfo ? lowerFrameInfo->time : 0;
  unsigned upperFrameTime = upperFrameInfo ? upperFrameInfo->time : 0;

  upperIsNew |= upperFrameTime != lastUpperFrameTime && (upperFrameTime >= lastAcceptedTime || SystemCall::getMode() == SystemCall::logFileReplay);
  lowerIsNew |= lowerFrameTime != lastLowerFrameTime && (lowerFrameTime >= lastAcceptedTime || SystemCall::getMode() == SystemCall::logFileReplay);
  lastUpperFrameTime = upperFrameTime;
  lastLowerFrameTime = lowerFrameTime;

  // Begin a new frame if either there is data for a new one left over from
  // the previous frame or we have two from which we can choose.
  if(acceptNext || (upperIsNew && lowerIsNew))
  {
    // We always switch between upper and lower
    isUpper ^= true;
    if(isUpper)
    {
      lastAcceptedTime = upperFrameTime;
      upperIsNew = false;

      // The other frame can only be used if not older than the current one.
      lowerIsNew &= upperFrameTime <= lowerFrameTime;
    }
    else
    {
      lastAcceptedTime = lowerFrameTime;
      lowerIsNew = false;

      // The other frame can only be used if not older than the current one.
      upperIsNew &= lowerFrameTime <= upperFrameTime;
    }

    // The other frame should also be processed immediately if there is one left.
    acceptNext = (isUpper && lowerIsNew) || (!isUpper && upperIsNew);
    return true;
  }
  else
  {
    // Wait for another frame to arrive before one can be picked.
    // However, a new frame is accepted when replaying logs.
    if(LogDataProvider::exists() && LogDataProvider::isFrameDataComplete()
       && SystemCall::getMode() == SystemCall::logFileReplay)
      return true;
  }

  return false;
}

void Cognition::beforeModules()
{
  // if the modules do not finish executing in this number of ms the watchdog
  // timer will fire and interrupt the process. Modules should complete in < 33 ms
  // so 1 second should be more than enough time. If we exceed that, something
  // is badly wrong.
  if (params.watchdogEnabled)
    watchdog.setTimeout(params.watchdogTimeout); // FIXME - SimRobot with 7v7 takes a long time to start up and a 1sec timeout causes the watchdog to fire
}

void Cognition::afterModules()
{
  // cancel the watchdog since we got here before it expired and shut us down
  watchdog.cancelTimeout();

  Blackboard& blackboard = Blackboard::getInstance();

  // NOTE: in the following it seems the representations will exist whether the
  // module that provides them is active or not, so the second check is
  // needed to see which module has actually filled in the representation
  // (sendIfNeeded) in order to decide how to proceed

  if (blackboard.exists("TeamMessage2024OutputGenerator") &&
           static_cast<const TeamMessage2024OutputGenerator &>(blackboard["TeamMessage2024OutputGenerator"])
               .sendIfNeeded)
  {
    BH_TRACE_MSG("before TeamMessage2024OutputGenerator.sendIfNeeded()");
    static_cast<const TeamMessage2024OutputGenerator &>(blackboard["TeamMessage2024OutputGenerator"]).sendIfNeeded();
  }
}

bool Cognition::afterFrame()
{
  // If there is already a frame waiting to be processed, do not wait for the next one to arrive.
  return !acceptNext;
}
