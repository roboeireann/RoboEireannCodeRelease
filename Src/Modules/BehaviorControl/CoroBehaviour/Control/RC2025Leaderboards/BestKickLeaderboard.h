/**
* @file BestKickLeaderboardAssitant.h
*
* A simple behaviour that will walk up and kick the ball at a selected target
* The targets include the center of the field, the opponent penalty spot and
* the opponents right(from robot perspective) corner spot.
*
* @author Nathan Ewnetu
*/

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"


namespace CoroBehaviour
{
namespace RC2025
{
namespace Leaderboards
{

  // =========================================================================

  CRBEHAVIOUR(BestKickTask)
  {
    CRBEHAVIOUR_INIT(BestKickTask)
    {
    }

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      while (getCoroDuration() < 3000)
      {
        lookLeftAndRightTask(true, 90_deg);
        commonSkills.stand();
        CR_YIELD();
      }

      getKickDistance();

      // OK button pressed, go and kick
      while (!gotoBallAndKickTask.isSuccess())
      {
        gotoBallAndKickTask(/* angle */ getKickAngle(), KickInfo::forwardLeftLonger, /* kickAlignPrecisely */ false,
                            kickDistance,
                            /* preStepAllowed */ true,
                            /* turnKickAllowed */ false);
        CR_YIELD();
      }

      while (true)
      {
        commonSkills.standLookForward();
        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(FieldBall);

    CommonSkills commonSkills{env};
    HeadSkills headSkills{env};
    GotoBallAndKickTask gotoBallAndKickTask{env};
    MotionSkills motionSkills{env};
    LookLeftAndRightTask lookLeftAndRightTask{env};
    MotionRequest::ObstacleAvoidance obstacleAvoidance;

    const Vector2f ballOffset = {-210.f, -50.f}; // Ideal offset from ball
    Vector2f kickTargetOnField;
    float kickDistance; ///< distance to kick the ball, set by the button pressed

    Angle getKickAngle()
    {
      // get vector from ball to target
      Vector2f ballVector = theRobotPose.toRobotCoordinates(kickTargetOnField) - theFieldBall.positionRelative;
      return ballVector.angle();
    }

    void getKickDistance()
    {
      //  kickTargetOnField = {3228,2280};
      //  kickDistance = 8919.64f;

      // Opponent Penalty Spot Target
      kickTargetOnField = {3200, 0};
      kickDistance = 7100.f;

      // Halfway point
      // kickTargetOnField = {0, 0};
      // kickDistance = 3900.f;

      // Corner Target
      // kickTargetOnField = {4500, 3000};
      // kickDistance = 8919.64f;
    }
  };

} // Leaderboards
} // RC2025
} // CoroBehaviour2025

