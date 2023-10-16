/**
 * @file GroundContactDetector.cpp
 * Implementation of a module that detects ground contact based on FSR measurements.
 * @author Thomas Röfer
 */

#include "GroundContactDetector.h"
#include "Platform/SystemCall.h"

MAKE_MODULE(GroundContactDetector, sensing);

void GroundContactDetector::update(GroundContactState& groundContactState)
{
  // don't do any ground contact detection until a few secs after startup
  if (startupTime == 0)
    startupTime = theFrameInfo.time;

  if (theFrameInfo.getTimeSince(startupTime) < 1500)
    return;

  if(groundContactState.contact) // was the robot in contact with the ground on last check?
  {
    if(theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] >= minPressureToKeepContact)
      lastTimeWithPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithPressure) < maxTimeWithoutPressure;
    if(!groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::say("High");
  }
  else // it wasn't in ground contact on last check, is it in contact with the ground now?
  {
    if(std::abs(theInertialSensorData.gyro.y()) > maxGyroYToRegainContact
       || theFsrSensorData.totals[Legs::left] + theFsrSensorData.totals[Legs::right] < minPressureToRegainContact
       || theFsrSensorData.totals[Legs::left] < minPressurePerFootToRegainContact
       || theFsrSensorData.totals[Legs::right] < minPressurePerFootToRegainContact)
      lastTimeWithoutPressure = theFrameInfo.time;
    groundContactState.contact = theFrameInfo.getTimeSince(lastTimeWithoutPressure) > minTimeWithPressure;
    if(groundContactState.contact && SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::say("Ground");
  }
}
