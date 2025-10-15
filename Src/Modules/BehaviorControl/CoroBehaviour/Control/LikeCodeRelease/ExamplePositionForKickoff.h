/**
 * @file PositionForKickOff.h
 *
 * This file implements a basic kickoff behaviour which at the moment
 * does exactly nothing.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/SoundSkills.h"

#include "Tools/Debugging/Annotation.h"

#include "Tools/Math/BHMath.h"
#include "Tools/Math/Pose2f.h"

namespace CoroBehaviour
{
  CRBEHAVIOUR(ExamplePositionForKickoffTask)
  {
    CRBEHAVIOUR_INIT(ExamplePositionForKickoffTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // No example behaviour implemented (i.e. matching the Code Release cards behaviour)
      commonSkills.say("Please implement a kick off behavior for me!");

      // await change of game state (handled by higher level task)
      while (true)
      {
        commonSkills.recordActivity(BehaviorStatus::codeReleasePositionForKickOff);
        commonSkills.standLookForward(true); // stand high
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills  {env};
  };

} // CoroBehaviour2022