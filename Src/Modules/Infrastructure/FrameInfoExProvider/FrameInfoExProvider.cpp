/**
 * @file FrameInfoExProvider.cpp
 *
 * A module that provides the FrameInfoEx representation.
 *
 * @author Rudi Villing
 */

#include "FrameInfoExProvider.h"
#include "Platform/SystemCall.h"
#include "TextLogging/TextLogging.h"


MAKE_MODULE(FrameInfoExProvider);

DECL_TLOGGER(tlogger, "FrameInfoExProvider", TextLogging::INFO);

void FrameInfoExProvider::update(FrameInfoEx& theFrameInfoEx)
{
  theFrameInfoEx.frameNumber = ++frameNumber;
  theFrameInfoEx.prevFrameTime = prevFrameTime;
  theFrameInfoEx.frameInterval = prevFrameTime ? theFrameInfo.time - prevFrameTime : 0;

  prevFrameTime = theFrameInfo.time;

  static_cast<FrameInfo&>(theFrameInfoEx) = theFrameInfo;


  // if ((theFrameInfoEx.frameInterval < 10) || (60 < theFrameInfoEx.frameInterval))
  if ((theFrameInfoEx.frameInterval > 60) && (SystemCall::getMode() == SystemCall::physicalRobot))
    TLOGW(tlogger, "unexpected frame interval: {}", theFrameInfoEx.frameInterval);

  TLOGV(tlogger, "number={}, time={}, prevTime={}, interval={}", theFrameInfoEx.frameNumber, theFrameInfoEx.time,
        theFrameInfoEx.prevFrameTime, theFrameInfoEx.frameInterval);
}