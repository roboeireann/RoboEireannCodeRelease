/**
 * Top level handling of player state (unstiff, prepare, active, etc)
 *
 * Converted to HybridBehaviour syntax from HandlePlayerState.cpp by B-Human
 */

#include "REHybridBehaviourModule.h"
#include "Tools/RECommon/EdgeTrigger.h"

DECL_TLOGGER(tlogger, "testGameStateTask", TextLogging::DEBUG);

#define ON_GAME_STATE_CHANGE(gcState_, textToSay_, ...)                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    gcState = gcState_;                                                                                                \
    if (gameStateChangeEdge.valueChanged())                                                                            \
    {                                                                                                                  \
      if (params.sayState)                                                                                             \
        sayTask({.text = textToSay_});                                                                                 \
      TLOGI(tlogger, __VA_ARGS__);                                                                                     \
    }                                                                                                                  \
  } while (false)


HSM_DEFINE((REHybridBehaviourModule, testGameStateTask),
    VARS((EdgeTrigger<GameState::State>, gameStateChangeEdge, GameState::numState), (std::string, gcState)),
    PARAMS((bool, sayState, true)))
{
  gameStateChangeEdge.update(theGameState.state);

  HSM_INITIAL_STATE(ignore)
  {
    HSM_TRANSITIONS { HSM_GOTO(execute); }
  }

  HSM_STATE(execute)
  {
    if (theGameState.isInitial())
      ON_GAME_STATE_CHANGE("INITIAL", "Initial", "INITIAL: gameState={}", TypeRegistry::getEnumName(theGameState.state));
    else if (theGameState.isMotionStoppedState())
      ON_GAME_STATE_CHANGE("MotionStopped", "motion stopped", "motionStoppedState: gameState={}", TypeRegistry::getEnumName(theGameState.state));
    else if (theGameState.isReady())
      ON_GAME_STATE_CHANGE("READY", "Ready", "READY: gameState={}", TypeRegistry::getEnumName(theGameState.state));
    else if (theGameState.isSet())
      ON_GAME_STATE_CHANGE("SET", "Set", "SET: gameState={}", TypeRegistry::getEnumName(theGameState.state));
    else if (theGameState.isPlaying())
      ON_GAME_STATE_CHANGE("PLAYING", "Playing", "PLAYING: gameState={}", TypeRegistry::getEnumName(theGameState.state));
    else if (theGameState.isFinished())
      ON_GAME_STATE_CHANGE("FINISHED", "Finished", "FINISHED: gameState={}", TypeRegistry::getEnumName(theGameState.state));

    THIS_TASK.trace("GC state = {}", gcState);
    THIS_TASK.trace("gameState = {}", TypeRegistry::getEnumName(theGameState.state));

    // ensure that we always set a head and motion request
    headPlayDeadTask();
    playDeadTask();
  }
}
