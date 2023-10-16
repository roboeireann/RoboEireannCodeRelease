/**
 * @file BasicSetStateTask.h
 *
 * This task implements a simple Set state behaviour.
 * Generally you would use a custom set state instead.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"


namespace CoroBehaviour
{
  CRBEHAVIOUR(BasicSetStateTask)
  {
    CRBEHAVIOUR_INIT(BasicSetStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.activityStatus(BehaviorStatus::set);

        if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
          headSkills.lookForward();
        else if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
        {
          if (!commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isGoalkeeper())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f(theFieldDimensions.xPosOwnPenaltyMark, 0),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.playsTheBall())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        }
        else // standard set state (at kickoff)
        {
          if (theTeamBehaviorStatus.role.playsTheBall())
            headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
                Vector2f::Zero(),
                theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
          else
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        }

        commonSkills.stand();

        if (theGameInfo.state != STATE_SET)
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(GameInfo);
    READS(TeamBehaviorStatus);
    READS(FieldDimensions);
    READS(BallSpecification);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
  };

} // CoroBehaviour2022