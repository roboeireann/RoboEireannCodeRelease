/**
 * @file: MotionSkills.h
 *
 * Basic (fairly low level) motion related skills (including stand, walking).
 * Generally there is little intelligence in handling the request.
 * For more intelligent walking, see the GotoSkills instead.
 *
 * @author: Rudi Villing
 * @author: Andy Lee Mitchell
 * @author: Aidan Colgan
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"
#include "Tools/Hysteresis.h"
#include "Tools/Streams/Enum.h"

#include "Tools/TextLogging.h"
#include "Tools/FmtCommonTypes.h"

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

    // isStanding
    // isWalking
    // isKicking
    //
    // use theMotionInfo.isMotion MotionPhase::stand or MotionPhase::walk 
    // or use theMotionInfo.isKicking

    bool isStanding() const { return theMotionInfo.isMotion(MotionPhase::stand); }
    bool isWalking() const { return theMotionInfo.isMotion(MotionPhase::walk); }


  private:
    BehaviourEnv& env;

    READS(LibCheck);
    READS(MotionInfo);
    READS(RobotPose);
    MODIFIES(MotionRequest);
  };

  ENUM(Avoidance,
  {,
    AVOIDANCE_NONE, // rough and no obstacle avoidance
    AVOIDANCE_ROUGH, // rough and libwalk avoidance
    AVOIDANCE_AUTO, // not rough, path planner or libwalk as needed
  });


  // ------------------------------------------------------------------------
  // resumable tasks to wrap functions above
  // ------------------------------------------------------------------------

  CRBEHAVIOUR(WalkToPoseTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseTask) {}

    void operator()(const Pose2f &target, float speed, Avoidance avoidance = AVOIDANCE_AUTO,
                    bool keepTargetRotation = false, bool standAtTarget = true,
                    Rangef positionThreshold = {20.f, 100.f}, Rangea angleThreshold = {3_deg, 10_deg})
    {
      const Pose2f speedPose = Pose2f(speed, speed, speed);
      const float targetDistance = target.translation.norm();
      const bool autoAvoidance = (avoidance == AVOIDANCE_AUTO); 

      const bool footContact = (autoAvoidance &&
                                (theFrameInfo.getTimeSince(theFootBumperState.status[Legs::left].lastContact) < 400 ||
                                 theFrameInfo.getTimeSince(theFootBumperState.status[Legs::right].lastContact) < 400));
      const bool leftArmContact =
          (autoAvoidance && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::left].timeOfLastContact) < 800));
      const bool rightArmContact =
          (autoAvoidance && (theFrameInfo.getTimeSince(theArmContactModel.status[Arms::right].timeOfLastContact) < 800));

      const bool obstacleContacted = footContact || leftArmContact || rightArmContact;

      const bool outsideOuterTargetThreshold =
          (targetDistance > positionThreshold.max) && (fabs(target.rotation) > angleThreshold.max);
      const bool insideInnerTargetThreshold =
          (targetDistance < positionThreshold.min) && (fabs(target.rotation) < angleThreshold.min);

      activePathPlannerComparator.update(targetDistance);
      const bool walkFarNeeded = autoAvoidance && activePathPlannerComparator.state;


      CRBEHAVIOUR_LOOP()
      {
        while (walkFarNeeded && !obstacleContacted)
        {
          CR_CHECKPOINT(walkFar);
          motionSkills.walkToPose(target, speedPose, getFarAvoidance(target, speedPose), keepTargetRotation);
          CR_YIELD();
        }

        while (!walkFarNeeded && !insideInnerTargetThreshold && !obstacleContacted)
        {
          CR_CHECKPOINT(walkNear);
          motionSkills.walkToPose(target, speedPose, getNearAvoidance(target, avoidance), keepTargetRotation);
          CR_YIELD();
        }

        // by definition insideTargetInnerThreshold implies !walkFarNeeded, so no need to check explicitly
        if (insideInnerTargetThreshold && !obstacleContacted)
        {
          CR_CHECKPOINT(reachingDestination);
          while (!outsideOuterTargetThreshold && !obstacleContacted)
          {
            // Note that the robot may already be standing in the right place, in which case we can just stand and exit
            if (standAtTarget && (motionSkills.isStanding() || (getCheckpointDuration() > 500)))
            {
              motionSkills.stand();
              CR_EXIT_SUCCESS();
            }
            else
            {
              motionSkills.walkToPose(target, speedPose, getNearAvoidance(target, avoidance), keepTargetRotation);
              if (getCheckpointDuration() > 500) // this handles the case where standAtTarget is false, but we've reached the target
                CR_EXIT_SUCCESS();
              else
                CR_YIELD();
            }            
          }
        }

        if (footContact)
        {
          CR_CHECKPOINT(footContactBackUp);
          while (getCheckpointDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
          {
            motionSkills.walkAtRelativeSpeed(Pose2f(0.f, -1.f, 0.f)); // backwards based on robot orientation
            CR_YIELD();
          }
        }
        else if (leftArmContact)
        {
          CR_CHECKPOINT(leftContactMoveRight);
          while (getCheckpointDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
          {
            motionSkills.walkAtRelativeSpeed(Pose2f(0.f, 0.f, -1.f)); // right based on robot orientation
            CR_YIELD();
          }
        }
        else if (rightArmContact)
        {
          CR_CHECKPOINT(righContactMoveLeft);
          while (getCheckpointDuration() < (autoAvoidance ? params.avoidanceTime : params.avoidanceTimeRough))
          {
            motionSkills.walkAtRelativeSpeed(Pose2f(0.f, 0.f, 1.f)); // left based on robot orientation
            CR_YIELD();
          }
        }

        // TLOGI(
        //     tlogger,
        //     "walkFarNeeded={}, insideInner={}, outsideOuter={}, obstacleContacted={} (foot={}, leftA={}, "
        //     "rightA={})",
        //     walkFarNeeded, insideInnerTargetThreshold, outsideOuterTargetThreshold, obstacleContacted,
        //     footContact, leftArmContact, rightArmContact);
        CR_END_OF_LOOP_CHECK(); // ensure at least one of the steps above has executed and yielded
      }
    }

  private:
    TextLogger tlogger = TextLogging::get("WalkToPoseTask");

    DEFINES_PARAMS(WalkToPoseTask, 
    {,
      /// Hysteresis: use pathPlanner when further than upper and LibWalk when closer than lower
      (HystThresholdsf)(900.f, 1000.f) activePathPlannerThresholds,  
      (unsigned)(1000) avoidanceTimeRough,
      (unsigned)(3000) avoidanceTime,
    });

    READS(RobotPose);
    READS(PathPlanner);
    READS(LibWalk);
    READS(ArmContactModel);
    READS(FrameInfo);
    READS(FootBumperState);

    MotionSkills motionSkills  {env};

    Comparatorf activePathPlannerComparator {params.activePathPlannerThresholds,false,true};

    MotionRequest::ObstacleAvoidance getFarAvoidance(const Pose2f &target, const Pose2f& speed)
    {
      return thePathPlanner.plan(theRobotPose.toFieldCoordinates(target), speed);
    }

    MotionRequest::ObstacleAvoidance getNearAvoidance(const Pose2f &target, Avoidance avoidance)
    {
      return theLibWalk.calcObstacleAvoidance(target, /* rough: */ avoidance != AVOIDANCE_AUTO,
                                              /* disableObstacleAvoidance */ avoidance == AVOIDANCE_NONE);
    }
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
        ACTGRAPH_FMT("target: {}", target);
        ACTGRAPH_FMT("robotPoseField: {}", theRobotPose);
        //AC-Extra info for debugging
        //addActivationGraphOutput(fmt::format("target.translation.squaredNorm() ={},sqr(positionThreshold) = {}",target.translation.squaredNorm(), sqr(positionThreshold)));
        //addActivationGraphOutput(fmt::format("fabs(target.rotation) = {},angleThreshold = {}",fabs(target.rotation),angleThreshold));

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
    READS(FieldDimensions);

    MotionSkills motionSkills  {env};
    HeadSkills headSkills {env};


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



//WalkToPoseAutoAvoidanceLookAroundTask
/**
 * Based on WalkToPoseAutoAvoidanceTask
 * Look Left and Right (Wide) whilst walking
 * 
*/



  CRBEHAVIOUR(WalkToPoseAutoAvoidanceLookAroundTask)
  {
    CRBEHAVIOUR_INIT(WalkToPoseAutoAvoidanceLookAroundTask) {}

    void operator()(const Pose2f &target, const Pose2f &speed,
                  float positionThreshold = 100.f, Angle angleThreshold = 10_deg)
    {
      CRBEHAVIOUR_LOOP()
      {

        CR_CHECKPOINT(walk_to_tactic_pose);
          //Set Head motion to look around as backing up to see beside it

          lookLeftAndRightTask();
          //Add Debugging Info for SimRobot
          ACTGRAPH_FMT("target = {}", target);
          ACTGRAPH_FMT("fabs rotation diff = {}", FmtAngle(std::fabs(target.rotation) - std::fabs(theRobotPose.rotation)));
          ACTGRAPH_FMT("fabs(theRobotPose.rotation) = {}", FmtAngle(std::fabs(theRobotPose.rotation)));

          motionSkills.walkToPose(target, speed, getObstacleAvoidance(target, speed),/*keepTargetRotation ->*/ true);

          if ((target.translation.squaredNorm() < sqr(positionThreshold)) && (std::fabs(target.rotation) < angleThreshold))
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();

      }
    }

  private:
    DEFINES_PARAMS(WalkToPoseAutoAvoidanceLookAroundTask, 
    {,
      (float)(1000.f) pathPlannerDistanceStart, /**< Hysteresis: If the target is further away than this distance, the path planner is used. */
      (float)(900.f) pathPlannerDistanceStop, /**< If the target is closer than this distance, LibWalk is used. */
      (float)(10000.f) frontWalkDurationMs, /**< How long to walk without checking behind-NEEDS TUNING*/
      (float)(1600.f) checkBackDurationMs, /**< How long to spend looking behind -NEEDS TUNING*/ 
      (float)(400.f) checkBehindThreshold, /**< How close to tacticPose before stop looking behind*/ 
    });
    READS(RobotPose);
    READS(PathPlanner);
    READS(LibWalk);

    MotionSkills motionSkills  {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};

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

  CRBEHAVIOUR(DribbleTask)
  {
    CRBEHAVIOUR_INIT(DribbleTask) {}

    /**
     * Dribble the ball.
     * @param targetDirection The (robot-relative) direction in which the ball should go
     * @param alignPrecisely Whether the robot should align more precisely than usual
     * @param kickPower The amount of power (in [0, 1]) that the kick should use
     * @param speed The walking speed as ratio of the maximum speed in [0, 1]
     * @param obstacleAvoidance The obstacle avoidance request
     * @param preStepAllowed Is a prestep for the InWalkKick allowed?
     * @param turnKickAllowed Does the forward kick not need to align with the kick direction?
     * @param directionPrecision The allowed deviation of the direction in which the ball should go. If default, the
     * WalkToBallAndKickEngine uses its own precision.
     */
    void operator()(Angle targetDirection, bool alignPrecisely, float kickPower,
                    const Pose2f &speed, const MotionRequest::ObstacleAvoidance &obstacleAvoidance,
                    bool preStepAllowed = true, bool turnKickAllowed = true,
                    const Rangea &directionPrecision = Rangea(0_deg, 0_deg))
    {
      CRBEHAVIOUR_BEGIN();

      // lastKickTimestamp = theMotionInfo.lastKickTimestamp;

      while (true)
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

        if (theMotionInfo.executedPhase == MotionPhase::walk)
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    // unsigned lastKickTimestamp; ///< used to track when a kick has actually been performed

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

      ACTGRAPH_FMT("start {}, odo {}, unwrapped {}",
                                           startRotation.fmt(), theOdometryData.rotation.fmt(),
                                           unwrappedOdometryRotation.fmt());
      ACTGRAPH_FMT("remaining {}, next {}", rotationRemaining.fmt(),
                                           nextRotation.fmt());

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