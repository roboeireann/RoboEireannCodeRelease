/**
 * @file KickTestsAssistant.h
 *
 * A simple behaviour that will walk up and kick the ball straight with a certain
 * target distance several times, giving voice prompts between each time.
 *
 * @author Meda Griniute
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BallSkills.h"

namespace RE
{
namespace v2024
{
  // =========================================================================

  CRBEHAVIOUR(KickOnceTask)
  {
    CRBEHAVIOUR_INIT(KickOnceTask) 
    {
      DECLARE_DEBUG_RESPONSE("behaviour:KickLengthCalibration:kickOnce");
    }

    void operator()(KickInfo::KickType kickType, float kickDistance)
    {
      // it can be a bit tricky to test button presses in the simulator.
      // (a) If you want to really test the button press you need to visualise the
      //     data for the EnhancedKeyStates, then modify the pressedDuration
      //     for headFront to be more than params.buttonPressThreshold (see below)
      //     and then briefly clikc the checkbox for pressed for headFront to be
      //     on and then off again.
      // (b) Otherwise, just write "dr behaviour:KickTests:kickOnce"
      //     in the console to set debugKickOnce to true for a single cycle which is enough
      //     to break the wait for button press while loop
      bool debugKickOnce = false;
      DEBUG_RESPONSE_ONCE("behaviour:KickTests:kickOnce")
      {
        debugKickOnce = true;
      }

      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(wait_for_button);

      if (SystemCall::getMode() == SystemCall::physicalRobot)
        commonSkills.say("Place the ball. Then touch the front head button.");

      // wait for the button to be pressed (or a debug request from SimRobot)
      while (!(debugKickOnce ||
               (theEnhancedKeyStates.pressed[KeyStates::headFront] &&
                theEnhancedKeyStates.pressedDuration[KeyStates::headFront] >= params.buttonPressThreshold)))
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }

      commonSkills.say(getSpeakableKick(kickType));

      // OK button pressed, go and kick

      CR_CHECKPOINT(kick);
      while (!gotoBallAndKickTask.isSuccess())
      {
        gotoBallAndKickTask(/* angle */ getKickAngle(), kickType, /* kickAlignPrecisely */ KickPrecision::notPrecise,
                            kickDistance,
                            /* preStepType */ PreStepType::allowed,
                            /* turnKickAllowed */ false);

        CR_YIELD();
      }

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
    READS(RobotPose);
    READS(FieldBall);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    Vector2f kickTargetOnField {6000.f, 0.f}; // straight towards opponent goal (if we are in goal mouth)

    Angle getKickAngle()
    {
      // get vector from ball to target
      Vector2f ballVector = theRobotPose.toRobotCoordinates(kickTargetOnField) - theFieldBall.positionRelative;
      return ballVector.angle();
    }

    std::string getSpeakableKick(KickInfo::KickType kickType)
    {
      std::string kickTypeStr;

      switch (kickType)
      {
        case KickInfo::forwardFastLeft: kickTypeStr = "forward fast left"; break;
        case KickInfo::forwardFastRight: kickTypeStr = "forward fast right"; break;
        case KickInfo::forwardFastLeftLong: kickTypeStr = "forward fast left long"; break;
        case KickInfo::forwardFastRightLong: kickTypeStr = "forward fast right long"; break;
        case KickInfo::walkForwardsLeft: kickTypeStr = "walk forwards left"; break;
        case KickInfo::walkForwardsRight: kickTypeStr = "walk forward right"; break;
        case KickInfo::walkForwardsLeftLong: kickTypeStr = "walk forwards left long"; break;
        case KickInfo::walkForwardsRightLong: kickTypeStr = "walk forwards right long"; break;
        case KickInfo::walkSidewardsRightFootToRight: kickTypeStr = "walk Sidewards Right Foot To Right"; break;
        case KickInfo::walkSidewardsLeftFootToLeft: kickTypeStr = "walk Sidewards Left Foot To Left"; break;
        case KickInfo::walkTurnRightFootToLeft: kickTypeStr = "walk Turn Right Foot To Left"; break;
        case KickInfo::walkTurnLeftFootToRight: kickTypeStr = "walk Turn Left Foot To Right"; break;
        case KickInfo::walkForwardStealBallLeft: kickTypeStr = "walk Forward Steal Ball Left"; break;
        case KickInfo::walkForwardStealBallRight: kickTypeStr = "walk Forward Steal Ball Right"; break;
        case KickInfo::walkForwardsRightAlternative: kickTypeStr = "walk Forwards Right Alternative"; break;
        case KickInfo::walkForwardsLeftAlternative: kickTypeStr = "walk Forwards Left Alternative"; break;
        default: kickTypeStr = "unknown kick type"; break;
      }

      return kickTypeStr;
    }
  };


  // =========================================================================

  CRBEHAVIOUR(KickTestsAssistantTask)
  {
    CRBEHAVIOUR_INIT(KickTestsAssistantTask) {}

    void operator()(void)
    {
      // do this regardless of place within coro body
      // commonSkills.recordActivity(BehaviorStatus::unknown);

      CRBEHAVIOUR_BEGIN();

      for (iKick = 0; iKick < params.kicksToTest.size(); iKick++)
      {
        kickOnceTask.reset();
        CR_AWAIT(kickOnceTask.isSuccess(), kickOnceTask(params.kicksToTest[iKick], params.testDistance));
      }

      if (SystemCall::getMode() == SystemCall::physicalRobot)
        commonSkills.say("All done.");

      while (true)
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(KickTestTask,
    {,
      (float)(2000.f) testDistance, ///< the test distance for kicks
      (std::vector<KickInfo::KickType>)({
        // KickInfo::forwardFastLeft,
        // KickInfo::forwardFastRight,
        // KickInfo::forwardFastLeftLong,
        // KickInfo::forwardFastRightLong,
        // KickInfo::walkForwardsLeft,
        // KickInfo::walkForwardsRight,
        // KickInfo::walkForwardsLeftLong,
        // KickInfo::walkForwardsRightLong,
        KickInfo::walkSidewardsRightFootToRight,
        KickInfo::walkSidewardsLeftFootToLeft,
        KickInfo::walkTurnRightFootToLeft,
        KickInfo::walkTurnLeftFootToRight,
        KickInfo::walkForwardStealBallLeft,
        KickInfo::walkForwardStealBallRight,
        // KickInfo::walkForwardsRightAlternative,
        // KickInfo::walkForwardsLeftAlternative,
      }) kicksToTest,
    });

    CommonSkills commonSkills {env};

    KickOnceTask kickOnceTask {env};

    size_t iKick = 0;
  };

} // namespace v2024
} // namespace RE