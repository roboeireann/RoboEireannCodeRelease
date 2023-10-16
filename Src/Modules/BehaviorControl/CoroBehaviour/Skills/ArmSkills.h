/**
 * @file: ArmSkills.h
 *
 * Skills that result in arm motion
 * 
 * These skills are adapted from various arm related skills in the
 * BH2021 code release to fit the Coro behaviour framework
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Platform/SystemCall.h"


namespace CoroBehaviour
{

  struct ArmSkills
  {
    ArmSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions (not tasks)
    // ------------------------------------------------------------------------

    // these functions are adapted from various ArmRequest skills in the
    // BH2019/2021 code releases

    /// point specific arm at a local point relative to the robot
    /// see also PointArmTask and PointAnyArmTask
    void pointArm(const Vector3f& localPoint, Arms::Arm arm)
    {
      theArmMotionRequest.armMotion[arm] = ArmMotionRequest::pointAt;
      theArmMotionRequest.pointToPointAt = localPoint;
      theLibCheck.setArm(arm);
    }

    /**
     * Perform a key frame motion with a one of the arms. (The other arm is
     * left as-is or can be moved independently with a separate call to
     * keyFrameSingleArm.)
     * @param motion The motion that the arm should execute
     * @param arm The arm that should execute the motion
     * @param fast true means states should not be interpolated
     */
    bool keyFrameArm(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast = false)
    {
      theArmMotionRequest.armMotion[arm] = ArmMotionRequest::keyFrame;
      theArmMotionRequest.armKeyFrameRequest.arms[arm].motion = motion;
      theArmMotionRequest.armKeyFrameRequest.arms[arm].fast = fast;
      theLibCheck.setArm(arm);

      return theArmMotionInfo.isKeyframeMotion(arm, motion);
    }

    /**
     * Execute the same key frame motion with both arms.
     * @param motion The motion that the arm should execute
     * @param fast true means states should not be interpolated
     */
    bool keyFrameBothArms(ArmKeyFrameRequest::ArmKeyFrameId motion, bool fast = false)
    {
      bool ret1 = keyFrameArm(motion, Arms::left, fast);
      bool ret2 = keyFrameArm(motion, Arms::right, fast);
      return ret1 && ret2;
    }


  private:
    BehaviourEnv& env;

    READS(LibCheck);
    READS(ArmMotionInfo);
    MODIFIES(ArmMotionRequest);
  };

  // ------------------------------------------------------------------------
  // resumable tasks
  // ------------------------------------------------------------------------

  /**
   * Point somewhere using specific arm.
   * Note: instead of using this task you can just use ArmSkills::pointArm
   * directly if you wish.
   */
  CRBEHAVIOUR(PointArmTask)
  {
    CRBEHAVIOUR_INIT(PointArmTask) {}

    /**
     * Point somewhere using specifed arm
     * @param localPoint The point in robot-relative coordinates
     * @param arm The arm that shall be used for pointing
     */
    void operator()(const Vector3f& localPoint, Arms::Arm arm)
    {
      CRBEHAVIOUR_LOOP()
      {
        armSkills.pointArm(localPoint, arm);
        CR_YIELD();
      }
    }

  private:
    ArmSkills armSkills  {env};
  };

  /**
   * Point somewhere using either arm.
   * Which arm is chosen by the function based on where the robot needs to point.
   */
  CRBEHAVIOUR(PointAnyArmTask)
  {
    CRBEHAVIOUR_INIT(PointAnyArmTask) {}

    /**
     * Point somewhere using either arm (chosen by the skill itself)
     * @param localPoint The point in robot-relative coordinates
     */
    void operator()(const Vector3f& localPoint)
    {
      CRBEHAVIOUR_BEGIN();

      threshold = 0.f;

      while (true)
      {
        if (localPoint.y() > threshold)
        {
          armSkills.pointArm(localPoint, Arms::left);
          threshold = -50.f; // hysteresis
        }
        else
        {
          armSkills.pointArm(localPoint, Arms::right);
          threshold = 50.f; // hysteresis
        }

        CR_YIELD();
      }
    }

  private:
    ArmSkills armSkills  {env};

    float threshold; /**< The current threshold to change the left/right arm decision. */
  };



  /**
   * Reacts to arm contact on the specified arm by taking the arm away.
   */
  CRBEHAVIOUR(ArmContactAnyArmTask)
  {
    CRBEHAVIOUR_INIT(ArmContactAnyArmTask) {}

    /**
     * Reacts to arm contact on the specified arm by taking the arm away.
     * @param arm The arm for which the contact reaction shall be done
     */
    void operator()()
    {
      CRBEHAVIOUR_BEGIN();

      timesOfLastAction[Arms::left] = timesOfLastAction[Arms::right] = 0;
      timesOfLastContact[Arms::left] = timesOfLastContact[Arms::right] = 0;


      while (true)
      {
        moveArmIfContact(Arms::left);
        moveArmIfContact(Arms::right);
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(ArmContactAnyArmTask,
    {,
      (int)(6000) stayBackTime, /**< The duration for which an arm stays back after contact. */
      (int)(2000) deadTime, /**< The duration for which new contacts are ignored after the arm has been back. */
    });

    READS(FrameInfo);
    READS(ArmContactModel);

    ArmSkills armSkills  {env};

    unsigned timesOfLastContact[Arms::numOfArms]; /**< The last time when a contact has been detected that should be reacted upon. */
    unsigned timesOfLastAction[Arms::numOfArms]; /**< The last time when an arm was taken back by this skill. */


    void moveArmIfContact(Arms::Arm arm)
    {
      if (SystemCall::getMode() == SystemCall::simulatedRobot)
        return;
      if (theArmContactModel.status[arm].contact &&
          (theArmContactModel.status[arm].pushDirection == ArmContactModel::backward ||
           theArmContactModel.status[arm].pushDirection == ArmContactModel::left ||
           theArmContactModel.status[arm].pushDirection == ArmContactModel::right) &&
          theFrameInfo.getTimeSince(timesOfLastAction[arm]) >= params.deadTime)
        timesOfLastContact[arm] = theFrameInfo.time;

      if (theFrameInfo.getTimeSince(timesOfLastContact[arm]) < params.stayBackTime)
      {
        armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm);
        timesOfLastAction[arm] = theFrameInfo.time;
      }
    }
  };



  /**
   * Reacts to nearby obstacles by taking the specified arm away.
   */
  CRBEHAVIOUR(ArmObstacleAvoidanceTask)
  {
    CRBEHAVIOUR_INIT(ArmObstacleAvoidanceTask) {}

    /**
     * Reacts to nearby obstacles by taking the specified arm away.
     * @param whichArm The arm for which the obstacle detection/reaction shall be done
     */
    void operator()(Arms::Arm whichArm)
    {
      CRBEHAVIOUR_BEGIN();

      arm = whichArm; // the arm can only be changed when the task has been reset

      while (true)
      {
        CR_CHECKPOINT(noAvoidanceNeeded);
        CR_AWAIT(shouldDoAvoidance(arm), CR_NOP());

        CR_CHECKPOINT(doAvoidance);
        CR_WHILE(shouldDoAvoidance(arm), armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm));

        CR_CHECKPOINT(finishAvoidance);
        CR_WHILE(getCheckpointDuration() < params.finishTime, armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm));

        CR_CHECKPOINT(deadTime);
        CR_WHILE(getCheckpointDuration() < params.deadTime, CR_NOP());
      }
    }

  private:
    DEFINES_PARAMS(ArmObstacleAvoidanceTask,
    {,
      (unsigned)(1000) finishTime,  /**< [ms] The duration for which an arm stays back after obstacle no longer detected. */
      (unsigned)(500) deadTime,     /**< [ms] The duration for which new contacts are ignored after the arm has been back. */
      (float)(400.f) distance, /**< [mm] Only obstacles closer than this are considered. */
    });

    READS(FrameInfo);
    READS(ObstacleModel);
    READS(MotionInfo);
    READS(FallDownState);

    ArmSkills armSkills  {env};

    Arms::Arm arm; // the arm to use

    bool shouldDoAvoidance(Arms::Arm arm) const
    {
      if (theMotionInfo.executedPhase != MotionPhase::walk || theFallDownState.state != FallDownState::upright)
        return false;

      const float distanceSquared = sqr(params.distance);
      // This sign makes the inequality in which it is used applicable for left as well as right.
      const float sign = arm == Arms::left ? 1.f : -1.f;
      for(const Obstacle& obstacle : theObstacleModel.obstacles)
        if((obstacle.left.squaredNorm() < distanceSquared && sign * obstacle.left.angle() > pi_8) ||
          (obstacle.center.squaredNorm() < distanceSquared && sign * obstacle.center.angle() > pi_8) ||
          (obstacle.right.squaredNorm() < distanceSquared && sign * obstacle.right.angle() > pi_8))
          return true;

      return false;
    }
  };


  /**
   * Reacts to nearby obstacles on the right/left by taking the appropriate arm away.
   */
  CRBEHAVIOUR(ArmObstacleAvoidanceAnyArmTask)
  {
    CRBEHAVIOUR_INIT(ArmObstacleAvoidanceAnyArmTask) {}

    /**
     * Reacts to nearby obstacles on the right/left by taking the appropriate arm away.
     */
    void operator()()
    {
      CR_LOOP()
      {
        leftArmObstacleAvoidanceTask(Arms::left);
        rightArmObstacleAvoidanceTask(Arms::right);
        CR_YIELD();
      }
    }

  private:
    ArmObstacleAvoidanceTask leftArmObstacleAvoidanceTask {env};
    ArmObstacleAvoidanceTask rightArmObstacleAvoidanceTask {env};
  };

} // CoroBehaviour