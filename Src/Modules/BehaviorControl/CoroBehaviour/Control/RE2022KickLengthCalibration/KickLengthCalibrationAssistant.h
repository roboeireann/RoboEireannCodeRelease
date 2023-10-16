/**
 * @file kickLengthCalibrationAssistant.h
 *
 * A simple behaviour that will walk up and kick the ball straight with a certain
 * target distance several times, giving voice prompts between each time.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

namespace CoroBehaviour
{
namespace RE2022
{
  // =========================================================================

  CRBEHAVIOUR(KickOnceTask)
  {
    CRBEHAVIOUR_INIT(KickOnceTask) {}

    void operator()(KickInfo::KickType kickType, float kickDistance)
    {
      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(wait_for_button);

      commonSkills.say("Please measure the previous kick distance if any.");
      commonSkills.say("When ready, place the ball. Then touch the front head button.");

      while (!(theEnhancedKeyStates.pressedDuration[KeyStates::headFront] >= params.buttonPressThreshold))
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }

      commonSkills.say(getSpeakableKick(kickType, kickDistance));

      // OK button pressed, go and kick

      CR_CHECKPOINT(kick);
      while (!gotoBallAndKickTask.isSuccess())
      {
        gotoBallAndKickTask(/* angle */ 0, kickType, /* kickAlignPrecisely */ true, kickDistance,
                            /* preStepAllowed */ true,
                            /* turnKickAllowed */ false);
        CR_YIELD();
      }

      // CR_CHECKPOINT(measure);
      // commonSkills.say("Please measure the kick distance.");
      // while (getCheckpointDuration() < params.measureDurationMs)
      // {
      //   commonSkills.standLookForward();
      //   CR_YIELD();
      // }

      commonSkills.standLookForward();
      CR_EXIT_SUCCESS();
    }

  private:
    DEFINES_PARAMS(KickOnceTask,
    {,
      (unsigned)(500) buttonPressThreshold, ///< Used as a comparator to how much the front head button was pressed for
      (unsigned)(3000) measureDurationMs, ///< some time to measure the kick distance
    });
    
    READS(EnhancedKeyStates);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    std::string getSpeakableKick(KickInfo::KickType kickType, float kickDistance)
    {
      std::string kickTypeStr;

      switch (kickType)
      {
        default:
        case KickInfo::forwardFastLeft: kickTypeStr = "forward fast left"; break;
        case KickInfo::forwardFastRight: kickTypeStr = "forward fast right"; break;
        case KickInfo::forwardFastLeftLong: kickTypeStr = "forward fast left long"; break;
        case KickInfo::forwardFastRightLong: kickTypeStr = "forward fast right long"; break;
        case KickInfo::walkForwardsLeft: kickTypeStr = "walk forwards left"; break;
        case KickInfo::walkForwardsRight: kickTypeStr = "walk forward right"; break;
        case KickInfo::walkForwardsLeftLong: kickTypeStr = "walk forwards left long"; break;
        case KickInfo::walkForwardsRightLong: kickTypeStr = "walk forwards right long"; break;
      }

      return fmt::format("{}, {} millimetres.", kickTypeStr, kickDistance);
    }
  };


  // =========================================================================

  CRBEHAVIOUR(KickLengthCalibrationAssistantTask)
  {
    CRBEHAVIOUR_INIT(KickLengthCalibrationAssistantTask) {}

    void operator()(void)
    {
      // do this regardless of place within coro body
      commonSkills.activityStatus(BehaviorStatus::unknown);

      CRBEHAVIOUR_BEGIN();
      
      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastLeft, params.testDistance));
      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastRight, params.testDistance));

      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastLeft, params.testDistance));
      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastRight, params.testDistance));

      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastLeft, params.testDistance));
      kickOnceTask.reset();
      CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastRight, params.testDistance));

      // kickOnceTask.reset();
      // CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastLeftLong, params.testDistance));
      // kickOnceTask.reset();
      // CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::forwardFastRightLong, params.testDistance));
      // kickOnceTask.reset();
      // CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::walkForwardsLeftLong, params.testDistance));
      // kickOnceTask.reset();
      // CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(KickInfo::walkForwardsRightLong, params.testDistance));

      commonSkills.say("All done.");
      while (true)
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(KickLengthCalibrationAssistantTask,
    {,
      (float)(2000.f) testDistance, ///< the test distance for kicks
    });

    CommonSkills commonSkills {env};

    KickOnceTask kickOnceTask {env};
  };

} // RE2022
} // CoroBehaviour2022