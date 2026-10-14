/**
 * @file BasicReturnFromPenalized.h
 *
 * Simplified version of BH2021 LookAroundAfterPenaltyCard
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"


namespace RE
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
        // commonSkills.recordActivity(BehaviorStatus::lookAroundAfterPenalty);
        lookLeftAndRightTask(startLeft);
        // commonSkills.stand();
        motionSkills.walkAtRelativeSpeed(Pose2f(0.f, 0.5f, 0.f));

        if (theFrameInfo.getTimeSince(theExtendedGameState.timeWhenLastUnpenalized) >=
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

    READS(ExtendedGameState);
    READS(RobotPose);
    READS(FrameInfo);

    CommonSkills commonSkills  {env};
    MotionSkills motionSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask  {env};
  };

} // namespace RE