/**
 * @file: BallSkills.h
 * 
 * Skills related to ball control
 * 
 * @author: Rudi Villing
 * 
 * Note - many parts of this file are based on Skill implementations in the
 * BH2021 code release but re-written for the CoroBehaviour engine. 
 * The original source is noted where applicable. Refer to cross-referenced
 * files for original authors.
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BehaviorStatusSkills.h"


namespace CoroBehaviour
{

  // ------------------------------------------------------------------------
  // Tasks
  // ------------------------------------------------------------------------


  /**
   * manage the head movement while going to the ball to kick it
   * 
   * Based on BH2021 GotoBallHeadControl.cpp
   */
  CRBEHAVIOUR(GotoBallHeadControlTask)
  {
    CRBEHAVIOUR_INIT(GotoBallHeadControlTask) {}

    /**
     * control the head for a robot that is doing gotoBallAndKick
     * @param distanceToKickPose The distance to the kick pose (from which the ball can be kicked).
     * @param lookAtKickTarget Whether the robot is allowed to look at the kick target.
     * @param kickTargetRelative The kick target (only needed if the previous parameter is true).
     */
    void operator()(float distanceToKickPose, bool lookAtKickTarget = false, Vector2f kickTargetRelative = Vector2f::Zero())
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(lookActive);
        CR_WHILE(!shouldLookAtBall(distanceToKickPose, false) &&
                      !shouldLookAtKickTarget(distanceToKickPose, lookAtKickTarget, kickTargetRelative, false),
                  headSkills.lookActive(/* withBall: */ true));

        CR_CHECKPOINT(lookAtKickTarget);
        CR_WHILE(!shouldLookAtBall(distanceToKickPose, false) &&
                      shouldLookAtKickTarget(distanceToKickPose, lookAtKickTarget, kickTargetRelative, true),
                  headSkills.lookAtPoint((Vector3f() << kickTargetRelative, 0.f).finished()));

        CR_CHECKPOINT(lookAtBall);
        CR_WHILE(shouldLookAtBall(distanceToKickPose, true), headSkills.lookAtBall());
      }
    }

  private:
    struct Params
    {
      float lookAtBallDistanceLow = 250.f; /**< If the distance to the target becomes smaller than this, the head looks at the ball. */
      float lookAtBallDistanceHigh = 310.f; /**< If the distance to the target becomes larger than this, other things than the ball may be looked at. */
      float mayLookAtKickTargetDistanceLow = 500.f; /**< If the distance to the target becomes smaller than this, the head may look at the kick target. */
      float mayLookAtKickTargetDistanceHigh = 600.f; /**< If the distance to the target becomes larger than this, the head control switches back to look active. */
      float areaAroundBallRadiusLow = 400.f; /**< If an obstacle is in the circle around the ball with this radius, the head control switches back to look active. */
      float areaAroundBallRadiusHigh = 600.f; /**< If no obstacle is in the circle around the ball with this radius, the head looks at the kick target direction. */
    }
    params;

    READS(BallModel);
    READS(FieldBall);
    READS(ObstacleModel);

    HeadSkills headSkills {env};


    bool shouldLookAtBall(float distanceToKickPose, bool lookingAtBall)
    {
      // Once we start looking at the ball, use higher threshold to stop looking (prevents fast changes between looking at not)
      return distanceToKickPose < (lookingAtBall ? params.lookAtBallDistanceHigh : params.lookAtBallDistanceLow);
    }

    bool shouldLookAtKickTarget(float distanceToKickPose, bool lookAtKickTargetAllowed, const Vector2f& kickTargetRelative, bool lookingAtKickTarget)
    {
      if (!lookAtKickTargetAllowed) // not if not allowed
        return false;
      // Not unless sufficiently close to the target.
      else if (distanceToKickPose > (lookingAtKickTarget ? params.mayLookAtKickTargetDistanceHigh : params.mayLookAtKickTargetDistanceLow))
        return false;
      // Not if the kick target is too far to the side or behind
      else if (std::abs(kickTargetRelative.angle()) > (lookingAtKickTarget ? 90_deg : 60_deg))
        return false;
      // Not If the ball is moving
      else if (theBallModel.estimate.velocity.squaredNorm() > (lookingAtKickTarget ? sqr(50.f) : 0.f))
        return false;
      // Not if the ball hasn't been seen for a while
      else if (!theFieldBall.ballWasSeen(lookingAtKickTarget ? 900 : 100))
        return false;
      
      // Not if there is any obstacle (maybe opponent robot) close to the ball as it could move the ball, so the ball should be given priority.
      const float obstacleThreshold = sqr(lookingAtKickTarget ? params.areaAroundBallRadiusLow : params.areaAroundBallRadiusHigh);
      for (const Obstacle& obstacle : theObstacleModel.obstacles)
        if((obstacle.center - theFieldBall.positionRelative).squaredNorm() < obstacleThreshold)
          return false;

      // No reason not to look at the kick target.
      return true;
    }
  };


  /**
   * go to ball and kick it
   * 
   * Based on BH2021 GotoBallAndKickSkill.cpp
   */
  CRBEHAVIOUR(GotoBallAndKickTask)
  {
      CRBEHAVIOUR_INIT(GotoBallAndKickTask) {}

    /**
     * Walk to the ball and kick it.
     * @param targetDirection The direction to which the ball should be kicked in robot-relative coordinates
     * @param kickType The kick type that should be executed there
     * @param alignPrecisely Whether the robot should align more precisely than usual
     * @param length The desired length of the kick (works only for certain types of kicks)
     * @param preStepAllowed Is a prestep for the InWalkKick allowed?
     * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
     * @param speed The walking speed
     * @param directionPrecision The allowed deviation of the direction in which the ball should go. 
     *                           If default, the WalkToBallAndKickEngine uses its own precision.
     */
    void operator()(Angle targetDirection, KickInfo::KickType kickType, bool alignPrecisely = true,
                    float length = std::numeric_limits<float>::max(), bool preStepAllowed = true,
                    bool turnKickAllowed = true, const Pose2f &speed = Pose2f(1.f, 1.f, 1.f),
                    const Rangea &directionPrecision = Rangea(0_deg, 0_deg))
    {
      const Pose2f kickPose = Pose2f(targetDirection, theFieldBall.endPositionRelative)
                                  .rotate(theKickInfo[kickType].rotationOffset)
                                  .translate(theKickInfo[kickType].ballOffset);

      behaviorStatusSkills.walkingTo(kickPose.translation, 1.f);

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walkToBallFar);
        CR_WHILE(!shouldIgnoreObstacles(kickPose, false) && !shouldUseLibWalk(kickPose, false) &&
                      !walkToBallAndKickTask.isSuccess(),
                  walkToBall(targetDirection, kickType, alignPrecisely, length, preStepAllowed, turnKickAllowed, speed,
                            directionPrecision, kickPose, /* usePathPlanner: */ true, /* avoidObstacles: */ true));

        CR_CHECKPOINT(walkToBallNear);
        CR_WHILE(!shouldIgnoreObstacles(kickPose, false) && shouldUseLibWalk(kickPose, true) &&
                      !walkToBallAndKickTask.isSuccess(),
                  walkToBall(targetDirection, kickType, alignPrecisely, length, preStepAllowed, turnKickAllowed, speed,
                            directionPrecision, kickPose, /* usePathPlanner: */ false, /* avoidObstacles: */ true));

        CR_CHECKPOINT(walkToBallIgnoreObstacles);
        CR_WHILE(shouldIgnoreObstacles(kickPose, true) && !walkToBallAndKickTask.isSuccess(),
                  walkToBall(targetDirection, kickType, alignPrecisely, length, preStepAllowed, turnKickAllowed, speed,
                            directionPrecision, kickPose, /* usePathPlanner: */ false, /* avoidObstacles: */ false));

        // has the ball been kicked?
        // FIXME: what should we do if it has not been kicked?
        if (walkToBallAndKickTask.isSuccess())
        {
          postKickRotation = theKickInfo[theMotionInfo.lastKickType].postRotationOffset;

          // do we need to turn after the kick
          if (postKickRotation != 0_deg)
          {
            CR_CHECKPOINT(turnAfterKick);
            while (!(turnToAngleOdometryTask.isSuccess() || theFieldBall.ballWasSeen(getCheckpointDuration() - 300)))
            {
              behaviorStatusSkills.walkingTo(kickPose.translation, 0.f);

              remainingRotation = turnToAngleOdometryTask(postKickRotation);
              headSkills.lookAtAngles(remainingRotation, 23_deg);
            }
          }

          // at this point we've turned if necessary so generate our final output and signal success
          CR_CHECKPOINT(kickComplete);
          walkToBall(targetDirection, kickType, alignPrecisely, length, preStepAllowed, turnKickAllowed, speed,
                      directionPrecision, kickPose, /* usePathPlanner: */ false, /* avoidObstacles: */ false);
          CR_EXIT_SUCCESS();
        }
      }
    }

  private:
    LOADS_PARAMS(GotoBallAndKickTask, 
    {,
      (float) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
      (float) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
      (float) goalkeeperIgnoreObstaclesDistanceStop, /**< If the target is further away than this distance, the goalkeeper stop ignoring obstacles. */
      (float) goalkeeperIgnoreObstaclesDistanceStart, /**< If the target is closer than this distance, the goalkeeper will start ignoring obstacles. */
      (Rangef) forwardFastKickRange, ///< the min and max distance of forwardFastLeft/Right kicks (depends on location)
    });

    READS(FieldBall);
    READS(BallModel);
    READS(KickInfo);
    READS(MotionInfo);
    READS(RobotInfo);
    READS(PathPlanner);
    READS(RobotPose);
    READS(LibWalk);

    BehaviorStatusSkills behaviorStatusSkills  {env};
    HeadSkills headSkills {env};
    GotoBallHeadControlTask gotoBallHeadControlTask  {env};
    WalkToBallAndKickTask walkToBallAndKickTask  {env};
    TurnToAngleOdometryTask turnToAngleOdometryTask  {env};

    Angle postKickRotation;
    Angle remainingRotation;

    /**
     * return true if the robot is the goalie and should ignore obstacles now
     * @param started true if the robot is already ignoring obstacles, false otherwise
     */
    bool shouldIgnoreObstacles(const Pose2f& kickPose, bool started)
    {
      float goalieIgnoreObstaclesDistance =
          started ? params.goalkeeperIgnoreObstaclesDistanceStop : params.goalkeeperIgnoreObstaclesDistanceStart;

      return (theRobotInfo.isGoalkeeper() && (kickPose.translation.norm() < goalieIgnoreObstaclesDistance));
    }

    /**
     * return true if the robot should use libWalk. False implies the path planner should be used.
     * It is necessary to check shouldIgnoreObstacles independently of this.
     */
    bool shouldUseLibWalk(const Pose2f& kickPose, bool started)
    {
      float libWalkDistance = started ? params.switchToPathPlannerDistance : params.switchToLibWalkDistance;

      return (kickPose.translation.norm() < libWalkDistance);
    }

    void walkToBall(Angle targetDirection, KickInfo::KickType kickType, bool alignPrecisely, float length,
                        bool preStepAllowed, bool turnKickAllowed, const Pose2f &speed,
                        const Rangea &directionPrecision, const Pose2f &kickPose,
                        bool usePathPlanner, bool avoidObstacles)
    {
      const float kickPower = kickLengthToPower(kickType, length, targetDirection);
      auto obstacleAvoidance = usePathPlanner
                                    ? thePathPlanner.plan(theRobotPose * kickPose, speed)
                                    : theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, !avoidObstacles);
      Vector2f kickTarget = theFieldBall.endPositionRelative +
                            Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection);

      gotoBallHeadControlTask(kickPose.translation.norm(), /* lookAtKickTarget: */ true, kickTarget);
      walkToBallAndKickTask(targetDirection, kickType, alignPrecisely, kickPower, speed,
                                obstacleAvoidance, preStepAllowed, turnKickAllowed, directionPrecision);
    }

    /**
     * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
     * @param kickType The requested kick type.
     * @param length The requested length.
     * @return The resulting kick power (1 for kicks with constant length).
     */
    float kickLengthToPower(KickInfo::KickType kickType, float length, Angle direction) const
    {
      if(kickType == KickInfo::forwardFastRight || kickType == KickInfo::forwardFastLeft)
      {
        return Rangef::ZeroOneRange().scale(length, params.forwardFastKickRange);
      }
      else
      {
        Rangef useKickRange = theKickInfo[kickType].range;

        if (kickType == KickInfo::walkForwardsLeft || kickType == KickInfo::walkForwardsRight)
        {
          const bool isLeft = (kickType == KickInfo::KickType::walkForwardsLeft);
          const KickInfo::KickType turnKickType = (!isLeft ? KickInfo::walkTurnRightFootToLeft : KickInfo::walkTurnLeftFootToRight);

          const Angle useMaxKickAngle = -theKickInfo[turnKickType].rotationOffset;
          const Rangea angleClip(!isLeft ? 0_deg : useMaxKickAngle, !isLeft ? useMaxKickAngle : 0_deg);
          const float interpolation = Rangef::ZeroOneRange().limit(angleClip.limit(direction) /
                                                                    -theKickInfo[turnKickType].rotationOffset);

          useKickRange.min = (1 - interpolation) * theKickInfo[kickType].range.min +
                              interpolation * theKickInfo[turnKickType].range.min;

          useKickRange.max = (1 - interpolation) * theKickInfo[kickType].range.max +
                              interpolation * theKickInfo[turnKickType].range.max;
        }

        return std::min(1.f, std::max(0.f, length - useKickRange.min) / (useKickRange.max - useKickRange.min));
      }
    }
  };

} // CoroBehaviour
