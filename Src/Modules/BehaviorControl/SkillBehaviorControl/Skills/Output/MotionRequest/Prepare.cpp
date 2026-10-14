/**
 * @file Prepare.cpp
 *
 * This file implements the Prepare skill
 *
 * @author Rudi Villing
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) Prepare)
{
  theMotionRequest.motion = MotionRequest::prepare;
  theMotionRequest.energySavingWalk = false;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if (theMotionInfo.executedPhase == MotionPhase::prepare)
        goto done;
    }
  }

  target_state(done) {}
}
