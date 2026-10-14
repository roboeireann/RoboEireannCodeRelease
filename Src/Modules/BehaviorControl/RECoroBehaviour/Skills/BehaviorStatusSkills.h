/**
 * @file: BehaviorStatusSkills.h
 *
 * Record various behaviour status information
 * (Note: american spelling on file to match underlying BehaviorStatus representation)
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"


namespace RE
{

  struct BehaviorStatusSkills
  {
    BehaviorStatusSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions to call directly
    // ------------------------------------------------------------------------

    // // see BH2021 Activity.cpp
    // void recordActivity(BehaviorStatus::Activity activity)
    // {
    //   theBehaviorStatus.activity = activity;
    //   // theLibCheck.inc(LibCheck::activity);
    // }

    // see BH2021 PassTarget.cpp
    void recordPassTarget(int passTarget, const Vector2f& ballTarget = Vector2f::Zero())
    {
      theBehaviorStatus.passTarget = passTarget;
      theBehaviorStatus.shootingTo = ballTarget;
      // theLibCheck.inc(LibCheck::passTarget);
    }

    // for kicks that are not intended as a pass
    void recordKick(const Vector2f& ballTarget = Vector2f::Zero())
    {
      theBehaviorStatus.shootingTo = ballTarget;
      // theLibCheck.inc(LibCheck::passTarget); // although not strictly a pass, we use this to prevent conflicts
    }

    // see BH2021 RecordTargetAndSpeed.cpp
    void recordWalkingTo(const Vector2f& target, float speed = 1.f)
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

} // namespace RE
