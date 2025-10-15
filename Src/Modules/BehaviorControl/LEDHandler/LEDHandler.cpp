/**
 * @file LEDHandler.cpp
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author jeff
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing
 */

#include "LEDHandler.h"

#include <algorithm>



// rotate least significant n bits left by numPlaces - useful for rotating patterns on LEDs
// works with arbitrary nBits values, not just powers of 2
[[maybe_unused]] static inline unsigned rotateBitsLeft(unsigned value, unsigned nBits, unsigned numPlaces)
{
  unsigned valMask = ((0x1 << nBits) - 1);
  unsigned tmp = value & valMask;
  numPlaces %= nBits;

  return (value & ~valMask) | ((tmp << numPlaces) & valMask) | (tmp >> (nBits - numPlaces));
}

// rotate least significant n bits right by numPlaces - useful for rotating patterns on LEDs
[[maybe_unused]] static inline unsigned rotateBitsRight(unsigned value, unsigned nBits, unsigned numPlaces)
{
  unsigned valMask = ((0x1 << nBits) - 1);
  unsigned tmp = value & valMask;
  numPlaces %= nBits;

  return (value & ~valMask) | (tmp >> numPlaces) | ((tmp << (nBits - numPlaces)) & valMask);
}

static LEDColor rainbow[8] = {LEDColor::RED,  LEDColor::ORANGE_YELLOW, LEDColor::BRIGHT_YELLOW, LEDColor::GREEN,
                              LEDColor::CYAN, LEDColor::BLUE,          LEDColor::PURPLE,        LEDColor::MAGENTA};
                              

void LEDHandler::update(LEDRequest& ledRequest)
{
  //reset
  FOREACH_ENUM(LEDRequest::LED, led)
    ledRequest.ledStates[led] = LEDRequest::off;

  setRightEye(ledRequest);
  setLeftEye(ledRequest);
  setChestButton(ledRequest);
  setLeftFoot(ledRequest);
  setRightFoot(ledRequest);

  //update
  setRightEar(ledRequest);
  setLeftEar(ledRequest);
  setHead(ledRequest);
}

void LEDHandler::setRightEar(LEDRequest& ledRequest)
{
  //right ear -> battery

  int onLEDs =
      std::min(static_cast<int>(theSystemSensorData.batteryLevel * LEDRequest::numOfEarLeds), static_cast<int>(LEDRequest::numOfEarLeds));

  LEDRequest::LEDState ledState = theSystemSensorData.batteryCharging ? LEDRequest::fade2 : LEDRequest::on;

  for (int i = 0; i < onLEDs; ++i)
    ledRequest.setEarLeds(LEDRequest::right, i, 1.f, ledState);
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
  float bright = 1.f;

  //left ear -> connected players
  //          + GameController connection lost -> freaky blinking
  if(theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) > gameControllerTimeOut)
  {
    ledRequest.setLed(LEDRequest::earsLeft324Deg, bright, LEDRequest::blinking);
    ledRequest.setLed(LEDRequest::earsLeft144Deg, bright, LEDRequest::blinking2);
  }

  // note that we intentionally don't use else-if in the following code
  int numberOfConnectedTeammates = static_cast<int>(theTeamData.teammates.size());
  if(numberOfConnectedTeammates > 0)
  {
    ledRequest.setLed(LEDRequest::earsLeft0Deg, bright, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft36Deg, bright, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 1)
  {
    ledRequest.setLed(LEDRequest::earsLeft72Deg, bright, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft108Deg, bright, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 2)
  {
    ledRequest.setLed(LEDRequest::earsLeft180Deg, bright, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft216Deg, bright, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 3)
  {
    ledRequest.setLed(LEDRequest::earsLeft252Deg, bright, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft288Deg, bright, LEDRequest::on);
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest)
{
  if (theLogfileLoggerStatus.state == LogfileLogger::FINISH_LOGGING)
  {
    // unsigned num = (theFrameInfo.time >> 7) & 0b111; // count from 0..7 with an increment every 128 ms
    // unsigned ledMask = rotateBitsLeft(0b00010001, 8, num);
    // ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::MASK | ledMask, rainbow[num], LEDRequest::on);
    // ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::MASK | ~ledMask, LEDColor::NONE, LEDRequest::off);

    unsigned offset = (theFrameInfo.time >> 7) & 0b111; // count from 0..7 with an increment every 128 ms
    for (unsigned i = 0; i < LEDRequest::numOfEyeLeds; i++)
    {
      unsigned index = (i + offset) % 8;
      ledRequest.setEyeLeds(LEDRequest::left, index, rainbow[i], LEDRequest::on);
    }
  }
  else if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)   //no groundContact?
    ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::ALL_LEDS, LEDColor::ORANGE_YELLOW);
  else
  {
    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
    bool featureSeen = theFrameInfo.getTimeSince(theFieldFeatureOverview.combinedStatus.lastSeen) < 250;

    if(ballSeen && featureSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::ALL_LEDS, LEDColor::RED);
    else if(ballSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::ALL_LEDS, LEDColor::PINK);
    else if(featureSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDRequest::ALL_LEDS, LEDColor::BLUE);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest)
{
  if (theLogfileLoggerStatus.state == LogfileLogger::FINISH_LOGGING)
  {
    // unsigned num = (theFrameInfo.time >> 7) & 0b111; // count from 0..7 with an increment every 128 ms
    // unsigned ledMask = rotateBitsRight(0b00010001, 8, num);
    // ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::MASK | ledMask, rainbow[num], LEDRequest::on);
    // ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::MASK | ~ledMask, LEDColor::NONE, LEDRequest::off);

    unsigned offset = (theFrameInfo.time >> 7) & 0b111; // count from 0..7 with an increment every 128 ms
    for (unsigned i = 0; i < LEDRequest::numOfEyeLeds; i++)
    {
      unsigned index = (i - offset) % 8;
      ledRequest.setEyeLeds(LEDRequest::right, index, rainbow[i], LEDRequest::on);
    }
  }
  else if (theTeamBehaviorStatus.role.isBallPlayer())
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::RED);
  else if (theTeamBehaviorStatus.role.isGoalkeeper())
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::BLUE);
  else if (theTeamBehaviorStatus.role.supporterIndex() == 3)
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::CYAN);
  else if (theTeamBehaviorStatus.role.supporterIndex() == 2)
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::GREEN);
  else if (theTeamBehaviorStatus.role.supporterIndex() == 1)
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::BRIGHT_YELLOW);
  else if (theTeamBehaviorStatus.role.supporterIndex() == 0)
    ledRequest.setEyeLeds(LEDRequest::right, LEDRequest::ALL_LEDS, LEDColor::WHITE);
}

void LEDHandler::setHead(LEDRequest& ledRequest)
{
  constexpr float bright = 1.f;

  // logging status

  if (theLogfileLoggerStatus.state == LogfileLogger::FINISH_LOGGING)
  {
    unsigned offset = (theFrameInfo.time >> 7) % LEDRequest::numOfHeadLeds; // increment every 128 ms
    unsigned ledMask = rotateBitsLeft(0b000000001001, LEDRequest::numOfHeadLeds, offset);
    ledRequest.setHeadLeds(LEDRequest::MASK | ledMask, bright, LEDRequest::on);
    ledRequest.setHeadLeds(LEDRequest::MASK | ~ledMask, bright, LEDRequest::off);
    return;
  }


  // battery status
  // rear 2 LEDs fading in/out confirms that the battery is charging

  if (theSystemSensorData.batteryCharging)
  {
    ledRequest.setLed(LEDRequest::headRearLeft2, bright, LEDRequest::fade2);
    ledRequest.setLed(LEDRequest::headRearRight2, bright, LEDRequest::fade2);
  }

  // comms status
  // 2 front LEDs on => GC comms OK, alternate blinking => no GC comms
  // all the mid-LEDs come briefly on any time a packet is received from any teammate
  // - this is easier to see and debug in a game situation
  //
  // Previously we did the following but not anymore!
  // individual LEDs rear and side switch on each time a packet received.
  // - The LED stays on constant for 1000ms and then fades over the following 2000ms
  // - LEDs are for players 1,2,3 on the left and 4,5 on the right (working 
  //   from back to front on each side)

  if (theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) > gameControllerTimeOut)
  {
    ledRequest.setLed(LEDRequest::headFrontLeft1, bright, LEDRequest::blinking2);
    ledRequest.setLed(LEDRequest::headFrontRight1, bright, LEDRequest::blinking);
  }
  else
  {
    ledRequest.setLed(LEDRequest::headFrontLeft1, bright, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headFrontRight1, bright, LEDRequest::on);
  }

  for (auto teammate : theTeamData.teammates)
  {
    // we want LED to switch on fully for 250 ms and then to fade out over the remaining 1000-250 = 750 ms
    float value = 1.f - Rangei(0,1000).limit(theFrameInfo.getTimeSince(teammate.timeWhenLastPacketReceived) - 250) / 750.f;

    // switch (teammate.number)
    // {
    //   case 1: ledRequest.setLed(LEDRequest::headRearLeft1, LEDRequest::on, value); break;
    //   case 2: ledRequest.setLed(LEDRequest::headRearLeft0, LEDRequest::on, value); break;
    //   case 3: ledRequest.setLed(LEDRequest::headMiddleLeft0, LEDRequest::on, value); break;
    //   case 4: ledRequest.setLed(LEDRequest::headRearRight1, LEDRequest::on, value); break;
    //   case 5: ledRequest.setLed(LEDRequest::headRearRight0, LEDRequest::on, value); break;
    //   default: ledRequest.setLed(LEDRequest::headMiddleRight0, LEDRequest::on, value); break;
    // }

    // switch on all the LEDs for any teammate message received (rather than one LED per teammate)
    ledRequest.setLed(LEDRequest::headRearLeft1, value, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headRearLeft0, value, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headMiddleLeft0, value, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headRearRight1, value, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headRearRight0, value, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headMiddleRight0, value, LEDRequest::on);
  }

}

void LEDHandler::setChestButton(LEDRequest& ledRequest)
{
  switch(theGameInfo.playerMode)
  {
    case GameInfo::unstiff:
      ledRequest.setChestLeds(LEDColor::BLUE, LEDRequest::slowDoubleBlink);
      break;
    case GameInfo::calibration:
      ledRequest.setChestLeds(LEDColor::MAGENTA);
      break;
    case GameInfo::active:
    default:
      if (theGameInfo.isPenalized())
        ledRequest.setChestLeds(LEDColor::RED);
      else
        switch (theGameInfo.state)
        {
          case STATE_STANDBY:
            ledRequest.setChestLeds(LEDColor::CYAN);
            break;
          case STATE_READY:
            ledRequest.setChestLeds(LEDColor::BLUE);
            break;
          case STATE_SET:
            ledRequest.setChestLeds(LEDColor::ORANGE_YELLOW);
            break;
          case STATE_PLAYING:
            ledRequest.setChestLeds(LEDColor::GREEN);
            break;
          case STATE_INITIAL:
          case STATE_FINISHED:
            // Chest LEDs off - don't need to do anything
            break;
        }
  }
}

void LEDHandler::setLeftFoot(LEDRequest& ledRequest)
{
  // not sure if we need to deal with only field player colours or goalie colour also.
  // The foot button interface is obsolete, so we'll go with field player colours only
  // for now.
  // Potentially needs revision later.
  switch(theGameInfo.ourTeam().fieldPlayerColor)
  {
    case TEAM_ORANGE:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::ORANGE_YELLOW); 
      break;
    case TEAM_RED:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::RED); 
      break;
    case TEAM_WHITE:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::WHITE); 
      break;
    case TEAM_YELLOW:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::BRIGHT_YELLOW);
      break;
    case TEAM_GREEN:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::GREEN); 
      break;
    case TEAM_PURPLE:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::PURPLE); 
      break;
    case TEAM_BLUE:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::BLUE); 
      break;
    case TEAM_GRAY:
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::GRAY); 
      break;
    case TEAM_BROWN: // more a darker yellow
      ledRequest.setFootLeds(LEDRequest::left, LEDColor::BROWN); 
      break;
  }
}

void LEDHandler::setRightFoot(LEDRequest& ledRequest)
{
  if (theGameInfo.state == STATE_INITIAL && theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT &&
      theGameInfo.isOurKick())
    ledRequest.setFootLeds(LEDRequest::right, LEDColor::GREEN);
  else if (theGameInfo.state == STATE_INITIAL && theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT &&
           !theGameInfo.isOurKick())
    ledRequest.setFootLeds(LEDRequest::right, LEDColor::BRIGHT_YELLOW);
  else if (theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) < gameControllerTimeOut &&
          theGameInfo.state <= STATE_SET &&
          theGameInfo.isOurKick())
    ledRequest.setFootLeds(LEDRequest::right, LEDColor::WHITE);
}

MAKE_MODULE(LEDHandler, behaviorControl);
