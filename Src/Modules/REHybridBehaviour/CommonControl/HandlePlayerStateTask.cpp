/**
 * Top level handling of player state (unstiff, prepare, active, etc)
 *
 * Converted to HybridBehaviour syntax from HandlePlayerState.cpp by B-Human
 */

#include "REHybridBehaviourModule.h"


DECL_TLOGGER(tlogger, "handlePlayerStateTask", TextLogging::DEBUG);

#define ON_STATE_CHANGE(textToSay_, ...)                                                                               \
  if (THIS_TASK.stateChanged())                                                                                        \
  {                                                                                                                    \
    if (params.sayStates)                                                                                              \
      sayTask({.text = textToSay_});                                                                                   \
    TLOGI(tlogger, __VA_ARGS__);                                                                                       \
  }

HSM_DEFINE((REHybridBehaviourModule, handlePlayerStateTask),
    PARAMS((bool, sayStates, true)))
{
  HSM_GLOBAL_TRANSITIONS
  {
    switch(theGameState.playerState)
    {
      case GameState::unstiff:
        HSM_GOTO(unstiff);
      case GameState::prepare:
        HSM_GOTO(prepare);
      case GameState::calibration:
        HSM_GOTO(calibration);
      case GameState::active:
        if(theExtendedGameState.wasPenalized())
        {
          HBS_ANNOTATION("Unpenalized");
          SystemCall::say("Not penalized");
        }
        if (theGameState.stopped)
          HSM_GOTO(activeStopped);
        else
          HSM_GOTO(active);
      default:
        if (theGameState.isPenalized())
        {
          if (!theExtendedGameState.wasPenalized())
          {
            HBS_ANNOTATION("Penalized {}", TypeRegistry::getEnumName(theGameState.playerState));
            SystemCall::say("Penalized");
          }
          HSM_GOTO(penalized);
        }
        else        
          FAIL_FMT("Unknown player state: {}", TypeRegistry::getEnumName(theGameState.playerState));
      }
  }

  HSM_INITIAL_STATE(active)
  {
    ON_STATE_CHANGE("active", "active: PlayerState {}, GameState {}",
                    TypeRegistry::getEnumName(theGameState.playerState), TypeRegistry::getEnumName(theGameState.state));

    // if we end up here (in the initial state) then the top level fallback will proceed to the next task
  }

  HSM_STATE(activeStopped)
  {
    ON_STATE_CHANGE("activeStopped", "activeStopped: PlayerState {}, GameState {}",
                    TypeRegistry::getEnumName(theGameState.playerState), TypeRegistry::getEnumName(theGameState.state));

    // LookForward();
    // Stand({.high = true});
  }

  HSM_STATE(unstiff)
  {
    ON_STATE_CHANGE("un-stiff", "unstiff");
    
    headPlayDeadTask();
    playDeadTask();
  }

  HSM_STATE(prepare)
  {
    ON_STATE_CHANGE("prepare", "prepare");

    lookAtAnglesTask({.pan = theJointAngles.angles[Joints::headYaw], .tilt = theJointAngles.angles[Joints::headPitch]}); 
    prepareTask();
  }

  HSM_STATE(calibration)
  {
    ON_STATE_CHANGE("calibration", "calibration");

    // CalibrationControl();
  }

  HSM_STATE(penalized)
  {
    if (THIS_TASK.stateChanged())
      TLOGD(tlogger, "penalized");

    lookForwardTask();
    standTask({.high = true});
  }
}
