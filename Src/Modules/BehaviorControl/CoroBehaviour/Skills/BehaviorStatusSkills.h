/**
 * @file: BehaviorStatusSkills.h
 *
 * Record various behaviour status information
 * (Note: american spelling on file to match underlying BehaviorStatus representation)
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"


namespace CoroBehaviour
{

  struct BehaviorStatusSkills
  {
    BehaviorStatusSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions to call directly
    // ------------------------------------------------------------------------

    // see BH2021 Activity.cpp
    void activity(BehaviorStatus::Activity activity)
    {
      theBehaviorStatus.activity = activity;
      theLibCheck.inc(LibCheck::activity);
    }

    // see BH2021 PassTarget.cpp
    void passTarget(int passTarget, const Vector2f& ballTarget = Vector2f::Zero())
    {
      theBehaviorStatus.passTarget = passTarget;
      theBehaviorStatus.shootingTo = ballTarget;
      theLibCheck.inc(LibCheck::passTarget);
    }

    // see BH2021 RecordTargetAndSpeed.cpp
    void walkingTo(const Vector2f& target, float speed = 1.f)
    {
      ASSERT(speed >= 0.f);
      ASSERT(speed <= 1.f);
      theBehaviorStatus.walkingTo = target;
      theBehaviorStatus.speed = speed * theWalkingEngineOutput.maxSpeed.translation.x();
    }

  private:
    BehaviourEnv& env;

    READS(LibCheck);
    READS(WalkingEngineOutput);
    MODIFIES(BehaviorStatus);
  };

} // CoroBehaviour
