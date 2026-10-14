/**
 * @file BasicSetStateTask.h
 *
 * This task implements a simple Set state behaviour.
 * Generally you would use a custom set state instead.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"


namespace RE
{
  CRBEHAVIOUR(BasicSetStateTask)
  {
    CRBEHAVIOUR_INIT(BasicSetStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.recordActivity(BehaviorStatus::set);

        if (theGameState.isPenaltyShootout()) // pseudo-penalized, waiting for turn in shootout
          headSkills.lookForward();
        else if (theGameState.isPenaltyKick())
        {
          if (!commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isGoalkeeper())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isBallPlayer())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        }
        else // standard set state (at kickoff)
        {
          if (theTeamBehaviorStatus.role.isBallPlayer())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f::Zero(),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        }

        commonSkills.stand();

        if (theGameState.isSet())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    // READS(REGameInfo);
    READS(GameState);
    READS(TeamBehaviorStatus);
    READS(FieldDimensions);
    READS(BallSpecification);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
  };

} // namespace RE