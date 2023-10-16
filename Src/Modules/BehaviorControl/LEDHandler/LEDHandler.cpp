/**
 * @file LEDHandler.cpp
 * This file implements a module that generates the LEDRequest from certain representations.
 * @author jeff
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing
 */

#include "LEDHandler.h"

#include <algorithm>

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
  // setBatteryLevelInEar(ledRequest, LEDRequest::earsRight0Deg);

  int onLEDs =
      std::min(static_cast<int>(theSystemSensorData.batteryLevel / 0.1f), static_cast<int>(LEDRequest::numOfEarLeds));

  LEDRequest::LEDState ledState = theSystemSensorData.batteryCharging ? LEDRequest::fade2 : LEDRequest::on;

  for (int i = 0; i <= onLEDs; ++i)
    // ledRequest.ledStates[baseLED + i] = LEDRequest::on;
    ledRequest.setEarLeds(LEDRequest::right, i, ledState, 1.f);
}

void LEDHandler::setLeftEar(LEDRequest& ledRequest)
{
  //left ear -> connected players
  //          + GameController connection lost -> freaky blinking
  if(theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) > gameControllerTimeOut)
  {
    ledRequest.setLed(LEDRequest::earsLeft324Deg, LEDRequest::blinking);
    ledRequest.setLed(LEDRequest::earsLeft144Deg, LEDRequest::blinking2);
  }

  int numberOfConnectedTeammates = static_cast<int>(theTeamData.teammates.size());
  if(numberOfConnectedTeammates > 0)
  {
    ledRequest.setLed(LEDRequest::earsLeft0Deg, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft36Deg, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 1)
  {
    ledRequest.setLed(LEDRequest::earsLeft72Deg, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft108Deg, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 2)
  {
    ledRequest.setLed(LEDRequest::earsLeft180Deg, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft216Deg, LEDRequest::on);
  }
  if(numberOfConnectedTeammates > 3)
  {
    ledRequest.setLed(LEDRequest::earsLeft252Deg, LEDRequest::on);
    ledRequest.setLed(LEDRequest::earsLeft288Deg, LEDRequest::on);
  }
}

void LEDHandler::setLeftEye(LEDRequest& ledRequest)
{
  //no groundContact
  if(!theGroundContactState.contact/* && (theFrameInfo.time & 512)*/)
    ledRequest.setEyeLeds(LEDRequest::left, LEDColor::ORANGE_YELLOW);
  else
  {
    bool ballSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 250;
    bool featureSeen = theFrameInfo.getTimeSince(theFieldFeatureOverview.combinedStatus.lastSeen) < 250;

    if(ballSeen && featureSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDColor::RED);
    else if(ballSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDColor::WHITE);
    else if(featureSeen)
      ledRequest.setEyeLeds(LEDRequest::left, LEDColor::BLUE);
  }
}

void LEDHandler::setRightEye(LEDRequest& ledRequest)
{
  if(theTeamBehaviorStatus.role.playsTheBall())
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::RED);
  else if(theTeamBehaviorStatus.role.isGoalkeeper())
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::BLUE);
  else if(theTeamBehaviorStatus.role.supporterIndex() == 0)
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::WHITE);
  else if(theTeamBehaviorStatus.role.supporterIndex() == 1)
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::ORANGE_YELLOW);
  else if(theTeamBehaviorStatus.role.supporterIndex() == 2)
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::GREEN);
  else if(theTeamBehaviorStatus.role.supporterIndex() == 3)
    ledRequest.setEyeLeds(LEDRequest::right, LEDColor::CYAN);
}

void LEDHandler::setHead(LEDRequest& ledRequest)
{
  // battery status
  // rear 2 LEDs fading in/out confirms that the battery is charging

  if(theSystemSensorData.batteryCharging)
  {
    // ledRequest.setLed(LEDRequest::headFrontLeft1, LEDRequest::fade2, 1.f);
    // ledRequest.setLed(LEDRequest::headFrontRight1, LEDRequest::fade2, 1.f);
    ledRequest.setLed(LEDRequest::headRearLeft2, LEDRequest::fade2, 1.f);
    ledRequest.setLed(LEDRequest::headRearRight2, LEDRequest::fade2, 1.f);
  }

  // comms status
  // 2 front LEDs on => GC comms OK, blinking => no GC comms
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
    // ledRequest.setLed(LEDRequest::headRearLeft2, LEDRequest::blinking);
    // ledRequest.setLed(LEDRequest::headRearRight2, LEDRequest::blinking);
    ledRequest.setLed(LEDRequest::headFrontLeft1, LEDRequest::blinking);
    ledRequest.setLed(LEDRequest::headFrontRight1, LEDRequest::blinking);
  }
  else
  {
    // ledRequest.setLed(LEDRequest::headRearLeft2, LEDRequest::on);
    // ledRequest.setLed(LEDRequest::headRearRight2, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headFrontLeft1, LEDRequest::on);
    ledRequest.setLed(LEDRequest::headFrontRight1, LEDRequest::on);
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
    ledRequest.setLed(LEDRequest::headRearLeft1, LEDRequest::on, value);
    ledRequest.setLed(LEDRequest::headRearLeft0, LEDRequest::on, value);
    ledRequest.setLed(LEDRequest::headMiddleLeft0, LEDRequest::on, value);
    ledRequest.setLed(LEDRequest::headRearRight1, LEDRequest::on, value);
    ledRequest.setLed(LEDRequest::headRearRight0, LEDRequest::on, value);
    ledRequest.setLed(LEDRequest::headMiddleRight0, LEDRequest::on, value);
  }

}

void LEDHandler::setChestButton(LEDRequest& ledRequest)
{
  switch(theRobotInfo.mode)
  {
    case RobotInfo::unstiff:
      ledRequest.setChestLeds(LEDRequest::blinking, LEDColor::BLUE);
      break;
    case RobotInfo::calibration:
      ledRequest.setChestLeds(LEDRequest::on, LEDColor::MAGENTA);
      break;
    case RobotInfo::active:
    default:
      if (theRobotInfo.penalty != PENALTY_NONE)
        ledRequest.setChestLeds(LEDRequest::on, LEDColor::RED);
      else
        switch (theGameInfo.state)
        {
          case STATE_READY:
            ledRequest.setChestLeds(LEDRequest::on, LEDColor::BLUE);
            break;
          case STATE_SET:
            ledRequest.setChestLeds(LEDRequest::on, LEDColor::ORANGE_YELLOW);
            break;
          case STATE_PLAYING:
            ledRequest.setChestLeds(LEDRequest::on, LEDColor::GREEN);
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
  // FIXME - potentially needs revision later.
  switch(theOwnTeamInfo.fieldPlayerColor)
  {
    case TEAM_ORANGE:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::ORANGE_YELLOW); 
      break;
    case TEAM_RED:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::RED); 
      break;
    case TEAM_WHITE:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::WHITE); 
      break;
    case TEAM_YELLOW:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::BRIGHT_YELLOW);
      break;
    case TEAM_GREEN:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::GREEN); 
      break;
    case TEAM_PURPLE:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::PURPLE); 
      break;
    case TEAM_BLUE:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::BLUE); 
      break;
    case TEAM_GRAY:
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::GRAY); 
      break;
    case TEAM_BROWN: // more a darker yellow
      ledRequest.setFootLeds(LEDRequest::left, LEDRequest::on, LEDColor::BROWN); 
      break;
  }
}

void LEDHandler::setRightFoot(LEDRequest& ledRequest)
{
  if (theGameInfo.state == STATE_INITIAL && theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT &&
      theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
    ledRequest.setFootLeds(LEDRequest::right, LEDRequest::on, LEDColor::GREEN);
  else if (theGameInfo.state == STATE_INITIAL && theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT &&
           theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber)
    ledRequest.setFootLeds(LEDRequest::right, LEDRequest::on, LEDColor::BRIGHT_YELLOW);
  else if (theFrameInfo.getTimeSince(theGameInfo.timeLastPacketReceived) < gameControllerTimeOut &&
          theGameInfo.state <= STATE_SET &&
          theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
    ledRequest.setFootLeds(LEDRequest::right, LEDRequest::on, LEDColor::WHITE);
}

MAKE_MODULE(LEDHandler, behaviorControl);
