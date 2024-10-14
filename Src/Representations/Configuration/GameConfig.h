/**
 * @file GameConfig.h
 * 
 * Configurable parameters related to the GameController and SPL rules that
 * may be used in multiple files
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Settings.h"
#include "Platform/BHAssert.h"

STREAMABLE(GameConfig,
{
  void verify()
  {
    ASSERT(playersPerTeam <= Settings::numPlayerNumbers);
  }

  /******* streamable members follow (note comma at end of this comment) *******/,

// All durations and delays are in ms.

  (unsigned) playersPerTeam,
  (int) kickoffReadyDuration, // duration of ready state leading to a kickoff
  (int) penaltyKickReadyDuration, // duration of ready state leading to a penalty kick
  (int) visualToReadyDelay, // delay after visual signal before READY is signalled by the GC
  (int) goalToReadyDelay, // delay after whistle for goal before READY is signalled by the GC
  (int) setToPlayingDelay, // delay after whistle in SET before PLAYING is signalled by the GC
  (int) kickoffDuration, // delay after whistle in SET before kickoff is deemed complete and ball free called (note: less than or equal setToPlayingDelay)
  (int) freeKickDuration, // after this time, ball free is called
  (int) penaltyKickDuration, // this is an in-game penalty kick; after this time, ball free is called
  (int) globalGameStuckTimeout, // robots must make some effort to explore the field to look for the ball in this time
  (bool) indirectKicksRequired,  // kickoff and all set plays except penalty kicks require a touch from a second teammate before scoring if true
  (int) messageBudget,
  (int) secsInHalf,
});
