/**
 * @file LedProviderK1.cpp
 * Determines the appropriate LED requests for the LedRequestK1 representation
 * based on factors such as GameState etc.
 * @author Rudi Villing
 */

#include "LedProviderK1.h"
#include "TextLogging/TextLogging.h"

#include <algorithm>


DECL_TLOGGER(tlogger, "LedProviderK1", TextLogging::WARNING);

void LedProviderK1::update(LedRequestK1& ledRequest)
{
  // default until overwritten in the update functions
  ledRequest.setFgPattern(LedColorK1::NONE, LedRequestK1::OFF);
  ledRequest.setBgPattern(LedColorK1::NONE, LedRequestK1::OFF);

  // process updates from lowest priority to highest - high priority updates
  // overwrite lower priority ones

  updateFromGameState(ledRequest);
  updateFromGroundState(ledRequest);
  updateFromTeammates(ledRequest);
  updateFromRobotSafetyState(ledRequest);
  updateFromKeyStates(ledRequest);

  MODIFY("module:LedProviderK1:ledRequest", ledRequest);

  if (ledRequest != lastRequest)
  {
    TLOGI(tlogger, "LedRequest changed: fg {{color=0x{:02x}{:02x}{:02x}, mode={}, pattern={:#010x}}} bg {{color=0x{:02x}{:02x}{:02x}, mode={}, pattern={:#010x}}}",
          ledRequest.fgColor.r, ledRequest.fgColor.g, ledRequest.fgColor.b, (int)ledRequest.fgMode, ledRequest.fgMask,
          ledRequest.bgColor.r, ledRequest.bgColor.g, ledRequest.bgColor.b, (int)ledRequest.bgMode, ledRequest.bgMask);
  }
  lastRequest = ledRequest;
}


void LedProviderK1::updateFromGameState(LedRequestK1 &ledRequest)
{
  // foreground flash if GC is connected
  if ((theGameState.gameControllerActive) && (((theFrameInfo.time - GC_OFFSET_MS) % LONG_PERIOD_MS) < FLASH_MS))
    ledRequest.setFgPattern(LedColorK1::WHITE, LedRequestK1::ON);

  // background colour based on state

  if (theGameState.playerState == GameState::unstiff)
    ledRequest.setBgPattern(LedColorK1::BLUE, LedRequestK1::SLOW_DOUBLE);
  else if (theGameState.playerState == GameState::prepare)
    ledRequest.setBgFade(LedColorK1::MID_GREEN, LedRequestK1::fade);
  else if (theGameState.playerState == GameState::calibration)
    ledRequest.setBgPattern(LedColorK1::MAGENTA, LedRequestK1::ON);
  else if (theGameState.isPenalized())
    ledRequest.setBgPattern(LedColorK1::RED, LedRequestK1::ON);
  else if (theGameState.isInitial(/*orStandby=*/false))
    ledRequest.setBgFade(LedColorK1::DARK_BLUE_GRAY, LedRequestK1::fade4);
  // else if(theGameState.state == GameState::standby)
  //   ledRequest.setBgPattern(LedColorK1::CYAN, LedRequestK1::ON);
  else if (theGameState.isReady())
    ledRequest.setBgPattern(LedColorK1::BLUE, LedRequestK1::ON);
  else if (theGameState.isSet())
    ledRequest.setBgPattern(LedColorK1::BRIGHT_YELLOW, LedRequestK1::ON);
  else if (theGameState.isPlaying())
    ledRequest.setBgPattern(LedColorK1::GREEN, LedRequestK1::ON);
}

void LedProviderK1::updateFromGroundState([[maybe_unused]] LedRequestK1 &ledRequest)
{
  // if (!theGroundContactState.contact)
  //   ledRequest.setFgPattern(LedColorK1::ORANGE_YELLOW, 0b00000000'00010000'00000000'00010000);
}

void LedProviderK1::updateFromTeammates(LedRequestK1 &ledRequest)
{
  // foreground flashes for each Teammate that has been communicating,
  // separated by TEAMMATE_INTERVAL_MS
  for (size_t i = 0; i < theGlobalTeammatesModel.teammates.size(); ++i)
  {
    if (((theFrameInfo.time - (TEAMMATE_OFFSET_MS + (i * TEAMMATE_INTERVAL_MS))) % LONG_PERIOD_MS) < FLASH_MS)
    {
      ledRequest.setFgPattern(LedColorK1::LIGHT_GRAY, LedRequestK1::ON);
      break;
    }
  }
}

void LedProviderK1::updateFromRobotSafetyState(LedRequestK1 &ledRequest)
{
  if (!theRobotSafetyState.hardwareConnected)
  {
    ledRequest.setFgPattern(LedColorK1::RED, LedRequestK1::BLINK);
    ledRequest.setBgPattern(LedColorK1::DARK_RED, LedRequestK1::ON);
  }
  else if (theRobotSafetyState.state == RobotSafetyState::safetyTripped)
  {
    ledRequest.setFgPattern(LedColorK1::BRIGHT_YELLOW, LedRequestK1::BLINK);
    ledRequest.setBgPattern(LedColorK1::DARK_YELLOW, LedRequestK1::ON);
  }
  else if (theRobotSafetyState.state == RobotSafetyState::resetPending)
  {
    ledRequest.setFgPattern(LedColorK1::DARK_YELLOW, LedRequestK1::FAST_BLINK);
    ledRequest.setBgPattern(LedColorK1::NONE, LedRequestK1::OFF);
  }
  // operational defaults back to normal LEDs in other functions
}


void LedProviderK1::updateFromKeyStates(LedRequestK1 &ledRequest)
{
  constexpr unsigned minLongClickMs = 500;

  // allow the pending safety transition to show since there isn't much colour contrast
  // to show the switch of state otherwise
  if (theRobotSafetyState.state == RobotSafetyState::resetPending)
    return;

  // otherwise show when a long press is in action
  if ((theEnhancedKeyStates.pressedDuration[KeyStates::k1_f1] > minLongClickMs) ||
      (theEnhancedKeyStates.pressedDuration[KeyStates::k1_stand] > minLongClickMs) ||
      (theEnhancedKeyStates.pressedDuration[KeyStates::k1_walk] > minLongClickMs))
  { // add a foreground flash that contrasts with the background once a key is being held down
    // (less contrast for colours that are already dark)
    float scale = (std::max({ledRequest.bgColor.r, ledRequest.bgColor.g, ledRequest.bgColor.b}) > 80) ? 0.35f : 0.7f;

    LedColorK1 fgColor = ledRequest.bgColor.scaled(scale);

    ledRequest.setFgPattern(fgColor, LedRequestK1::VERY_FAST_BLINK);
    // ledRequest.setFgPattern(LedColorK1::RED, LedRequestK1::VERY_FAST_BLINK);
  }
}




MAKE_MODULE(LedProviderK1);
