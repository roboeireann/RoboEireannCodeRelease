/**
 * @file GotoBallSkills.h
 *
 * Passing and shooting skills.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"


namespace CoroBehaviour
{
namespace RC2023
{

  // =====================================================================

  /**
   * Pass a ball to position chosen by the caller
   */
  CRBEHAVIOUR(GotoBallAndPassTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndPassTask) {}

    void operator()(const Vector2f& target)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_BEGIN();

      targetIsLeft = Random::bernoulli(); // choose a random initial target side for angles close

      while (true)
      {
        // try not to walk into goal
        if (theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField))
        {
          commonSkills.standLookActive();
          CR_EXIT_SUCCESS();
        }

        updateKickParams(target);
        gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely, kickDistance, /* preStepAllowed */ true,
                            /* turnKickAllowed */ false);

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndPassTask,
    {,
      (Rangef)(2000.f, 7000.f) forwardFastLongKickRange, ///< In this range the kick would reach the target
      (Rangef)(0.f, 6500.f) forwardFastKickRange, ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange, ///< In this range the kick would reach the target
      (Angle)(10_deg) minKickSectorSize,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;

    bool targetIsLeft = true; // default


    CommonSkills commonSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    void updateKickParams(const Vector2f& target)
    {
      kickDistance = 5000.f;//target.norm() + 500.f; // approximation wtih margin
      kickAngle = target.angle();

      // choose the kick type (based on distance and angle)

      if (kickAngle > 15_deg) // kicking with right foot (toward our left)
        targetIsLeft = true;
      else if (kickAngle < -15_deg) // kicking with left foot (toward our right)
        targetIsLeft = false;
      // else leave targetIsLeft at current value

      kickAlignPrecisely = true; // default unless changed below

      // how strong to make the kick?
      // if (obstacleSkills.isOpponentDangerNearBall())
      //   kickType = !targetIsLeft ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      // else if (params.forwardFastLongKickRange.isInside(kickDistance))
      // {
      //   kickAlignPrecisely = true;
      //   kickType = !targetIsLeft ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      // }
      if (params.forwardFastKickRange.isInside(kickDistance))
        kickType = !targetIsLeft ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else
        kickType = !targetIsLeft ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;

      addActivationGraphOutput(fmt::format("kickAngle = {}_deg, distance = {}", 
          toDegrees(kickAngle), kickDistance));
      addActivationGraphOutput(fmt::format("kickType = {}", TypeRegistry::getEnumName(kickType)));
    }
  };


  // ======================================================================
  /**
   * Pass a ball to position chosen by the caller
   */
  CRBEHAVIOUR(GotoBallAndShootTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndShootTask) {}

    void operator()(void)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_BEGIN();

      targetIsLeft = Random::bernoulli(); // choose initial side at random

      while (true)
      {
        // try not to walk into goal
        if (theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField))
        {
          commonSkills.standLookActive();
          CR_EXIT_SUCCESS();
        }

        updateKickParams();
        gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely, kickDistance);

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndShootTask,
    {,
      (Rangef)(2000.f, 5000.f) forwardFastLongKickRange, ///< In this range the kick would reach the target
      (Rangef)(1600.f, 2500.f) forwardFastKickRange, ///< In this range the kick would reach the target
      (Rangef)(800.f, 2000.f) walkForwardsLongKickRange, ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange, ///< In this range the kick would reach the target
      (Angle)(10_deg) minKickSectorSize,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;
    bool targetIsLeft = true;

    CommonSkills commonSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    void updateKickParams()
    {
      setGoalKickAngleAndDistance();
      setGoalKickType();
    }

    void setGoalKickAngleAndDistance()
    {
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGroundLine, 0.f};
      Vector2f goalCenter = theRobotPose.toRobotCoordinates(goalCenterOnField);

      kickAngle = goalCenter.angle();
      kickDistance = goalCenter.norm();

      // check for obstacle (goalkeeper) in goal and find best angle to shoot
      // (assuming goal is close enough)
      if (kickDistance < 6000.f) // FIXME HACK
      {
        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
        (void)kickSectors;
        Rangea bestGoalSector = obstacleSkills.getBestGoalSector(kickSectors);

        if (bestGoalSector.getSize() > params.minKickSectorSize)
        {
          kickAngle = (bestGoalSector.max + bestGoalSector.min) / 2; // mid angle = average
        }
      }

      kickDistance += 1500.f; // margin to ensure we shoot strongly

      addActivationGraphOutput(fmt::format("kickAngle = {}_deg", toDegrees(kickAngle)));
    }

    void setGoalKickType()
    {
      // choose the kick type (based on distance and angle)

      if (kickAngle > 15_deg) // kicking with right foot (toward our left)
        targetIsLeft = true;
      else if (kickAngle < -15_deg) // kicking with left foot (toward our right)
        targetIsLeft = false;
      // else leave ballIsLeft the same as in the previous cycle

      kickAlignPrecisely = false; // default unless changed below

      // how strong to make the kick?
      if (obstacleSkills.isOpponentDangerNearBall())
        kickType = !targetIsLeft ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      else if (params.forwardFastLongKickRange.isInside(kickDistance))
      {
        kickAlignPrecisely = true;
        kickType = !targetIsLeft ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      }
      else if (params.forwardFastKickRange.isInside(kickDistance))
        kickType = !targetIsLeft ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else
        kickType = !targetIsLeft ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
    }
  };


} // RC2023
} // CoroBehaviour2023
