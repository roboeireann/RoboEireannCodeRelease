/**
 * @file BasicReturnFromPenalized.h
 *
 * Simplified version of BH2021 LookAroundAfterPenaltyCard
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"


namespace CoroBehaviour
{
  /**
   * basic return from penalized - look around for a bit to get our bearings
   */
  CRBEHAVIOUR(BasicReturnFromPenalizedTask)
  {
    CRBEHAVIOUR_INIT(BasicReturnFromPenalizedTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // FIXME
      startLeft = Random::bernoulli();
      // startLeft = theTeamBallModel.isValid ? (theTeamBallModel.position.y() > 0.f) : Random::bernoulli();

      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::lookAroundAfterPenalty);
        lookLeftAndRightTask(startLeft);
        commonSkills.stand();

        if (theExtendedGameInfo.timeSinceLastPenaltyEnded >=
             ((theRobotPose.quality != RobotPose::poor) ? params.minDuration : params.maxDuration))
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(BasicReturnFromPenalizedTask, 
    {,
      (int)(500) minDuration, ///< Stay with this task for at least minDuration ms after a robot has been unpenalized.
      (int)(3000) maxDuration, ///< Leave the task once the robot has been unpenalized for more than this duration
    });

    bool startLeft;

    READS(ExtendedGameInfo);
    READS(RobotPose);

    CommonSkills commonSkills  {env};
    LookLeftAndRightTask lookLeftAndRightTask  {env};
  };

} // CoroBehaviour2022