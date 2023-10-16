/**
 * @file: MotionSkills.h
 *
 * Basic (fairly low level) motion related skills (including stand, walking).
 * Generally there is little intelligence in handling the request.
 * For more intelligent walking, see the GotoSkills instead.
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

namespace CoroBehaviour
{

  struct MotionSkills
  {
    MotionSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions (not tasks)
    // ------------------------------------------------------------------------

    // these functions are adapted from various MotionRequest skills in the
    // BH2019/2021 code releases

    /**
     * Turn stiffness off on all joints. (Only do this when the robot is
     * already in the safe sitting-on-haunches position!)
     */
    bool unstiff()
    {
      theMotionRequest.motion = MotionRequest::playDead;
      theLibCheck.inc(LibCheck::motionRequest);
      return theMotionInfo.executedPhase == MotionPhase::playDead;
    }

    /**
     * Make the robot stand.
     * @param high True if the knees should be straight (low energy),
     *             false for bent knees (ready to walk)
     */
    bool stand(bool high = false)
    {
      theMotionRequest.motion = MotionRequest::stand;
      theMotionRequest.standHigh = high;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.executedPhase == MotionPhase::stand;
    }

    bool walkAtAbsoluteSpeed(const Pose2f &speed)
    {
      theMotionRequest.motion = MotionRequest::walkAtAbsoluteSpeed;
      theMotionRequest.walkSpeed = speed;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.executedPhase == MotionPhase::walk;
    }

    bool walkAtRelativeSpeed(const Pose2f &speed)
    {
      theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
      theMotionRequest.walkSpeed = speed;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.executedPhase == MotionPhase::walk;
    }

    /**
     * This skill walks to a (relative) target.
     * @param target The target pose in robot-relative coordinates
     * @param speed The walking speed as ratio of the maximum speed in [0, 1]
     * @param obstacleAvoidance The obstacle avoidance request
     * @param keepTargetRotation Whether the target rotation should be headed for all the time (instead of allowing
     * motion to plan it)
     */
    bool walkToPose(const Pose2f &target, const Pose2f &speed,
                    const MotionRequest::ObstacleAvoidance &obstacleAvoidance, bool keepTargetRotation)
    {
      // FIXME - theRecordTargetAndSpeedSkill(p.target.translation, std::max(p.speed.translation.x(),
      // p.speed.translation.y()));

      theMotionRequest.motion = MotionRequest::walkToPose;
      theMotionRequest.walkTarget = target;
      theMotionRequest.walkSpeed = speed;
      theMotionRequest.keepTargetRotation = keepTargetRotation;
      theMotionRequest.obstacleAvoidance = obstacleAvoidance;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.executedPhase == MotionPhase::walk;
    }

    /**
     * walk to pose in Field (SPL) coordinates
     */
    bool walkToPoseOnField(const Pose2f &targetOnField, const Pose2f &speed,
                           const MotionRequest::ObstacleAvoidance &obstacleAvoidance, bool keepTargetRotation)
    {
      Pose2f targetRel = theRobotPose.inversePose + targetOnField;
      return walkToPose(targetRel, speed, obstacleAvoidance, keepTargetRotation);
    }

    /**
     * Dribble the ball.
     * @param targetDirection The (robot-relative) direction in which the ball should go
     * @param speed The walking speed as ratio of the maximum speed in [0, 1]
     * @param obstacleAvoidance The obstacle avoidance request
     * @param alignPrecisely Whether the robot should align more precisely than usual
     * @param kickPower The desired strength of the dribble kick
     * @param preStepAllowed Is a prestep for the InWalkKick allowed?
     * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
     * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the
     * WalkToBallAndKickEngine uses its own precision.
     */
    bool dribble(Angle targetDirection, const Pose2f &speed, const MotionRequest::ObstacleAvoidance &obstacleAvoidance,
                 bool alignPrecisely = false, float kickPower = 1.f, bool preStepAllowed = true,
                 bool turnKickAllowed = true, const Rangea &directionPrecision = Rangea(0_deg, 0_deg))
    {
      theMotionRequest.motion = MotionRequest::dribble;
      theMotionRequest.walkSpeed = speed;
      theMotionRequest.obstacleAvoidance = obstacleAvoidance;
      theMotionRequest.targetDirection = targetDirection;
      theMotionRequest.directionPrecision = directionPrecision;
      theMotionRequest.alignPrecisely = alignPrecisely;
      theMotionRequest.kickPower = kickPower;
      theMotionRequest.turnKickAllowed = turnKickAllowed;
      theMotionRequest.preStepAllowed = preStepAllowed;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.executedPhase == MotionPhase::walk;
    }


    /**
     * request a key frame motion
     * @param env the coro behaviour environment
     * @param id the id of the motion to execute
     * @param mirror whether or not the motion should be mirrored (to the opposite side of the robot body)
     * @returns true if motion is done (at least corresponding to the done state of BH2021 KeyFrameMotion skill)
     */
    bool keyframeMotion(KeyframeMotionRequest::KeyframeMotionID id, bool mirror = false)
    {
      theMotionRequest.motion = MotionRequest::keyframeMotion;
      theMotionRequest.keyframeMotionRequest.keyframeMotion = id;
      theMotionRequest.keyframeMotionRequest.mirror = mirror;
      theLibCheck.inc(LibCheck::motionRequest);

      return theMotionInfo.isKeyframeMotion(id, mirror);
    }

    /**
     * request a getUp motion
     * @param env the coro behaviour environment
     * @returns true if motion is done (at least corresponding to the done state of BH2021 GetUpEngine skill)
     */
    bool getUp()
    {
      theMotionRequest.motion = MotionRequest::getUp;
      theLibCheck.inc(LibCheck::motionRequest);
      return theMotionInfo.isMotion(MotionPhase::getUp);
    }

  private:
    BehaviourEnv& env;

    READS(LibCheck);
    READS(MotionInfo);
    READS(RobotPose);
    MODIFIES(MotionRequest);
  };

  // ------------------------------------------------------------------------
  // resumable tasks to wrap functions above
  // ------------------------------------------------------------------------

  CRBEHAVIOUR(WalkToPoseTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseTask) {}

    void operator()(const Pose2f &target, const Pose2f &speed,
                    const MotionRequest::ObstacleAvoidance &obstacleAvoidance, bool keepTargetRotation,
                    float positionThreshold, Angle angleThreshold = 180_deg)
    {
      CRBEHAVIOUR_LOOP()
      {
        motionSkills.walkToPose(target, speed, obstacleAvoidance, keepTargetRotation);

        if ((target.translation.squaredNorm() < sqr(positionThreshold)) && (fabs(target.rotation) < angleThreshold))
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    MotionSkills motionSkills  {env};
  };


  CRBEHAVIOUR(WalkToPoseAutoAvoidanceTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseAutoAvoidanceTask) {}

    void operator()(const Pose2f &target, const Pose2f &speed, bool keepTargetRotation = false,
                    float positionThreshold = 100.f, Angle angleThreshold = 10_deg)
    {
      CRBEHAVIOUR_LOOP()
      {
        motionSkills.walkToPose(target, speed, getObstacleAvoidance(target, speed), keepTargetRotation);

        if ((target.translation.squaredNorm() < sqr(positionThreshold)) && (fabs(target.rotation) < angleThreshold))
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(WalkToPoseAutoAvoidanceTask, 
    {,
      (float)(1000.f) pathPlannerDistanceStart, /**< Hysteresis: If the target is further away than this distance, the path planner is used. */
      (float)(900.f) pathPlannerDistanceStop, /**< If the target is closer than this distance, LibWalk is used. */
    });

    READS(RobotPose);
    READS(PathPlanner);
    READS(LibWalk);

    MotionSkills motionSkills  {env};

    bool usingPathPlanner = false;

    MotionRequest::ObstacleAvoidance getObstacleAvoidance(const Pose2f &target, const Pose2f& speed)
    {
      float targetDistance = target.translation.norm();

      if (targetDistance >= params.pathPlannerDistanceStart)
        usingPathPlanner = true;
      else if (targetDistance <= params.pathPlannerDistanceStop)
        usingPathPlanner = false;
      // otherwise leave usingPathPlanner as is to give a little bit of hysteresis for switching between the two

      return usingPathPlanner
                 ? thePathPlanner.plan(theRobotPose.toFieldCoordinates(target), speed)
                 : theLibWalk.calcObstacleAvoidance(target, /* rough: */ true, /* disableObstacleAvoidance */ false);
    }
  };



  CRBEHAVIOUR(WalkToPoseNoAvoidanceTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseNoAvoidanceTask) {}

    void operator()(const Pose2f &target, const Pose2f &speed, bool keepTargetRotation = false,
                    float positionThreshold = 100.f, Angle angleThreshold = 10_deg)
    {
      CRBEHAVIOUR_LOOP()
      {
        motionSkills.walkToPose(
            target, speed,
            theLibWalk.calcObstacleAvoidance(target, /* rough: */ true, /* disableObstacleAvoidance */ true),
            keepTargetRotation);

        if ((target.translation.squaredNorm() < sqr(positionThreshold)) && (fabs(target.rotation) < angleThreshold))
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    READS(LibWalk);

    MotionSkills motionSkills  {env};
  };




  CRBEHAVIOUR(WalkToBallAndKickTask)
  {
    CRBEHAVIOUR_INIT(WalkToBallAndKickTask) {}

    /**
     * Walk to the ball and kick it.
     * @param targetDirection The (robot-relative) direction in which the ball should go
     * @param kickType The type of kick that should be used
     * @param alignPrecisely Whether the robot should align more precisely than usual
     * @param kickPower The amount of power (in [0, 1]) that the kick should use
     * @param speed The walking speed as ratio of the maximum speed in [0, 1]
     * @param obstacleAvoidance The obstacle avoidance request
     * @param preStepAllowed Is a prestep for the InWalkKick allowed?
     * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
     * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the
     * WalkToBallAndKickEngine uses its own precision.
     */
    void operator()(Angle targetDirection, KickInfo::KickType kickType, bool alignPrecisely, float kickPower,
                    const Pose2f &speed, const MotionRequest::ObstacleAvoidance &obstacleAvoidance,
                    bool preStepAllowed = true, bool turnKickAllowed = true,
                    const Rangea &directionPrecision = Rangea(0_deg, 0_deg))
    {
      CRBEHAVIOUR_BEGIN();

      lastKickTimestamp = theMotionInfo.lastKickTimestamp;

      while (true)
      {
        theMotionRequest.motion = MotionRequest::walkToBallAndKick;
        theMotionRequest.walkSpeed = speed;
        theMotionRequest.obstacleAvoidance = obstacleAvoidance;
        theMotionRequest.targetDirection = targetDirection;
        theMotionRequest.directionPrecision = directionPrecision;
        theMotionRequest.kickType = kickType;
        theMotionRequest.kickPower = kickPower;
        theMotionRequest.alignPrecisely = alignPrecisely;
        theMotionRequest.preStepAllowed = preStepAllowed;
        theMotionRequest.turnKickAllowed = turnKickAllowed;
        theLibCheck.inc(LibCheck::motionRequest);

        if (theMotionInfo.lastKickTimestamp != lastKickTimestamp)
          CR_EXIT_SUCCESS(); // lastKickTimestamp indicates that the kick is complete!
        else
          CR_YIELD();
      }
    }

  private:
    unsigned lastKickTimestamp; ///< used to track when a kick has actually been performed

    READS(MotionInfo);
    READS(LibCheck);
    MODIFIES(MotionRequest);
  };

  // ==========================================================================

  CRBEHAVIOUR(TurnDirectionOdometryTask)
  {
    CRBEHAVIOUR_INIT(TurnDirectionOdometryTask) {}

    /**
     * Turns the robot on the spot by the specified angle (relative to starting 
     * angle using odometry only to avoid localisation jumps)
     * 
     * The direction of the turn is determined by the sign of the specified
     * angle - positive is anticlockwise, negative is clockwise. The angle
     * can represent more than one turn.
     * 
     * @param targetAngle The target angle relative to the robot's starting orientation - cannot be changed after initial call
     * @param tolerance The tolerance at which the targetAngle is considered reached
     */
    Angle operator()(Angle targetAngle, Angle tolerance = 5_deg)
    {
      CRBEHAVIOUR_BEGIN();

      startRotation = theOdometryData.rotation;
      unwrappedOdometryRotation = theOdometryData.rotation;

      while (true)
      {
        if (turnToAngle(targetAngle, tolerance)) 
          CR_EXIT_SUCCESS_VAL(rotationRemaining);
        else
          CR_YIELD_VAL(rotationRemaining);
      }
    }

  private:
    Angle startRotation; ///< The odometry based rotation when the robot started the task      
    Angle rotationRemaining; ///< the unnormalized remaining rotation to the targetAngle
    Angle nextRotation;
    Angle unwrappedOdometryRotation;

    MotionSkills motionSkills  {env};

    READS(OdometryData);
    READS(LibWalk);

    bool turnToAngle(Angle targetAngle, Angle tolerance)
    {
      // FIXME: this might be fragile - it depends on odometry changing by 
      // less than 180 degrees in any cognition cycle - this sounds reasonable, but...
      Angle delta = theOdometryData.rotation - unwrappedOdometryRotation; 
      if (std::abs(delta) < 180_deg)
        unwrappedOdometryRotation += delta;
      else if (delta < -180_deg)
        unwrappedOdometryRotation += (delta + 360_deg);
      else
        unwrappedOdometryRotation += (delta - 360_deg);


      rotationRemaining = startRotation + targetAngle - unwrappedOdometryRotation;
      nextRotation = Rangea(-90_deg, 90_deg).limit(rotationRemaining);


      const Pose2f target = Pose2f(nextRotation);
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(target, true, true);

      motionSkills.walkToPose(target, Pose2f(1.0f, 1.0f, 1.0f), obstacleAvoidance, false); // FIXME replace with walkToPointSkill

      addActivationGraphOutput(fmt::format("start {:.1f}_deg, odo {:.1f}_deg, unwrapped {:.1f}",
                                           startRotation.toDegrees(), theOdometryData.rotation.toDegrees(),
                                           unwrappedOdometryRotation.toDegrees()));
      addActivationGraphOutput(fmt::format("remaining {:.1f}_deg, next {:.1f}_deg", rotationRemaining.toDegrees(),
                                           nextRotation.toDegrees()));

      return std::abs(rotationRemaining) < tolerance;
    }
  };

  // ==========================================================================

  CRBEHAVIOUR(TurnToAngleOdometryTask)
  {
    CRBEHAVIOUR_INIT(TurnToAngleOdometryTask) {}

    /**
     * Turns the robot on the spot by the target angle (relative to starting 
     * angle using odometry only to avoid localisation jumps)
     * 
     * NOTE: that the turn direction is chosen to reach the angle as quickly as possible
     * which is not what you want if trying to do a full or nearly full rotation on
     * the spot.
     * 
     * @param targetAngle The target angle relative to the robot's starting orientation
     * @param tolerance The tolerance at which the targetAngle is considered reached
     */
    Angle operator()(Angle targetAngle, Angle tolerance = 5_deg)
    {
      CRBEHAVIOUR_BEGIN();

      startRotation = theOdometryData.rotation;

      while (true)
      {
        if (turnToAngle(targetAngle, tolerance)) 
          CR_EXIT_SUCCESS_VAL(rotation);
        else
          CR_YIELD_VAL(rotation);
      }
    }

  private:
    Angle startRotation; ///< The odometry based rotation when the robot started the task      
    Angle rotation; ///< the remaining rotation to the targetAngle

    MotionSkills motionSkills  {env};

    READS(OdometryData);
    READS(LibWalk);

    bool turnToAngle(Angle targetAngle, Angle tolerance)
    {
      rotation = Angle::normalize(startRotation + targetAngle - theOdometryData.rotation);
      
      const Pose2f target = Pose2f(rotation);
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(target, true, true);

      motionSkills.walkToPose(target, Pose2f(1.0f, 1.0f, 1.0f), obstacleAvoidance, false); // FIXME replace with walkToPointSkill

      return std::abs(rotation) < tolerance;
    }
  };


  CRBEHAVIOUR(TurnToPointOdometryTask)
  {
    CRBEHAVIOUR_INIT(TurnToPointOdometryTask) {}

    /**
     * Turns the robot on the spot by the target angle (relative to starting 
     * angle using odometry only to avoid localisation jumps)
     * 
     * @param targetAngle The target angle relative to the robot's starting orientation
     * @param tolerance The tolerance at which the targetAngle is considered reached
     */
    Angle operator()(const Vector2f& target, Angle tolerance = 5_deg)
    {
      CRBEHAVIOUR_BEGIN();

      while (true)
      {
        rotation = turnToAngleOdometryTask(target.angle(), tolerance);
        if (turnToAngleOdometryTask.isSuccess())
          CR_EXIT_SUCCESS_VAL(rotation);
        else
          CR_YIELD_VAL(rotation);
      }
    }

  private:
    Angle rotation;
    TurnToAngleOdometryTask turnToAngleOdometryTask  {env};
  };


} // CoroBehaviour