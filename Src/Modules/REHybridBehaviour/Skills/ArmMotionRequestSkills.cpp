/**
 * @file: ArmSkills.h
 *
 * Skills that result in arm motion
 * 
 * These skills are adapted from various arm related skills in the
 * RE2025/BH2025 code releases to fit the Hybrid behaviour framework
 *
 * @author: Rudi Villing
 */

#include "REHybridBehaviourModule.h"


// ------------------------------------------------------------------------
// stateless helper functions (not tasks)
// ------------------------------------------------------------------------

/**
  * Perform a key frame motion with a one of the arms. (The other arm is
  * left as-is or can be moved independently with a separate call to
  * keyFrameSingleArm.)
  * @param motion The motion that the arm should execute
  * @param arm The arm that should execute the motion
  * @param fast true means states should not be interpolated
  */
bool BehaviourContainer::keyFrameArmHelper(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast)
{
  theArmMotionRequest.armMotion[arm] = ArmMotionRequest::keyFrame;
  theArmMotionRequest.armKeyFrameRequest.arms[arm].motion = motion;
  theArmMotionRequest.armKeyFrameRequest.arms[arm].fast = fast;
  // theLibCheck.setArm(arm);

  return theArmMotionInfo.isKeyframeMotion(arm, motion);
}

/**
  * Execute the same key frame motion with both arms.
  * @param motion The motion that the arm should execute
  * @param fast true means states should not be interpolated
  */
bool BehaviourContainer::keyFrameArmsHelper(ArmKeyFrameRequest::ArmKeyFrameId motion, Arms::Arm arm, bool fast)
{
  bool ret1 = true;
  bool ret2 = true;

  if (arm == Arms::left || arm == Arms::bothArms)
    ret1 = keyFrameArmHelper(motion, Arms::left, fast);
  if (arm == Arms::right || arm == Arms::bothArms)
    ret2 = keyFrameArmHelper(motion, Arms::right, fast);

  return ret1 && ret2;
}


// ------------------------------------------------------------------------
// Tasks
// ------------------------------------------------------------------------

// point the arm (left, right, or anyArm) at a 3d point (robot coordinates)

CORO_DEFINE((BehaviourContainer, pointAtTask),
      ARGS((const Vector3f&, localPoint),
            (Arms::Arm, arm)),
      VARS((int, side, 0)))
{
  CORO_BEGIN()

  // 1 is left, -1 is right
  side = (args.arm == Arms::left || (args.arm == Arms::anyArm && args.localPoint.y() > 0.f)) ? 1 : -1;

  while (true)
  {
    if ((side == 1) && (args.localPoint.y() < -50.f))
      side = -1;
    else if ((side == -1) && (args.localPoint.y() > 50.f))
      side = 1;

    if (args.arm == Arms::left || side == 1)
      theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::pointAt;
    else
      theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::pointAt;

    theArmMotionRequest.pointToPointAt = args.localPoint;

    CORO_YIELD();
  }

  CORO_END()
}


// move the arms using a key frame motion (left, right, or bothArms)

CORO_DEFINE((BehaviourContainer, keyFrameArmsTask),
    ARGS((ArmKeyFrameRequest::ArmKeyFrameId, motion),
        (Arms::Arm, arm),
        (bool, fast)))
{
  CORO_BEGIN()

  while (true)
  {
    if (keyFrameArmsHelper(args.motion, args.arm, args.fast))
      CORO_YIELD_SUCCESS();
    else
      CORO_YIELD();
  }

  CORO_END()
}


// check for arm contact and move the arm if necessary (left, right, or bothArms)

CORO_DEFINE((BehaviourContainer, armContactTask),
    ARGS((Arms::Arm, arm)))
{
  CORO_BEGIN()

  while (true)
  {
    if (args.arm == Arms::left)
      armContactLeftTask();
    else if (args.arm == Arms::right)
      armContactRightTask();
    else // both
    {
      armContactLeftTask();
      armContactRightTask();
    }
    CORO_YIELD();
  }

  CORO_END()
}

// armContact delegator and delegate tasks - the delegators exist to maintain independent context/state
// for the left and right arm but all the work is done by the delegate which must be called using
// HBS_INVOKE_DELEGATE to ensure the caller context gets passed through

HSM_DEFINE_DELEGATOR((BehaviourContainer, armContactLeftTask))
{
  HSM_INITIAL_STATE(delegator)
  {
    HBS_INVOKE_DELEGATE(armContactArmDelegate, {.arm = Arms::left});
  }
}

HSM_DEFINE_DELEGATOR((BehaviourContainer, armContactRightTask))
{
  HSM_INITIAL_STATE(delegator)
  {
    HBS_INVOKE_DELEGATE(armContactArmDelegate, {.arm = Arms::right});
  }
}


HSM_DEFINE_DELEGATE((BehaviourContainer, armContactArmDelegate),
    ARGS((Arms::Arm, arm)),
    PARAMS((unsigned, stayBackMs, 6000), /**< The duration for which an arm stays back after contact. */
        (unsigned, refactoryMs, 2000))) /**< The duration for which new contacts are ignored after the arm has been back. */
{
  const bool hasContact = theArmContactModel.status[args.arm].contact
                          && (theArmContactModel.status[args.arm].pushDirection == ArmContactModel::backward
                              || theArmContactModel.status[args.arm].pushDirection == ArmContactModel::left
                              || theArmContactModel.status[args.arm].pushDirection == ArmContactModel::right);

  HSM_INITIAL_STATE(armNormal)
  {
    HSM_TRANSITIONS
    {
      if (SystemCall::getMode() == SystemCall::simulatedRobot)
        HSM_GOTO(ignoreInSimulation);
      else if (hasContact)
        HSM_GOTO(armBack);
    }
  }

  HSM_STATE(armBack)
  {
    HSM_TRANSITIONS
    {
      if (!hasContact)
        HSM_GOTO(keepBack);
    }

    keyFrameArmHelper(ArmKeyFrameRequest::back, args.arm);
  }

  HSM_STATE(keepBack)
  {
    HSM_TRANSITIONS
    {
      if (hasContact)
        HSM_GOTO(armBack);
      else if (THIS_TASK.stateDuration() > params.stayBackMs)
        HSM_GOTO(delay);
    }

    keyFrameArmHelper(ArmKeyFrameRequest::back, args.arm);
  }

  HSM_STATE(delay)
  {
    HSM_TRANSITIONS
    {
      if (THIS_TASK.stateDuration() > params.refactoryMs)
        goto armNormal;
    }
  }

  HSM_FAILURE_STATE(ignoreInSimulation) {}
}




// TODO - this still needs to be converted

// namespace RE
// {
//   // ------------------------------------------------------------------------
//   // resumable tasks
//   // ------------------------------------------------------------------------
// 
//   /**
//    * Reacts to arm contact on the specified arm by taking the arm away.
//    */
//   CRBEHAVIOUR(ArmContactAnyArmTask)
//   {
//     CRBEHAVIOUR_INIT(ArmContactAnyArmTask) {}
// 
//     /**
//      * Reacts to arm contact on the specified arm by taking the arm away.
//      * @param arm The arm for which the contact reaction shall be done
//      */
//     void operator()()
//     {
//       CRBEHAVIOUR_BEGIN();
// 
//       timesOfLastAction[Arms::left] = timesOfLastAction[Arms::right] = 0;
//       timesOfLastContact[Arms::left] = timesOfLastContact[Arms::right] = 0;
// 
// 
//       while (true)
//       {
//         moveArmIfContact(Arms::left);
//         moveArmIfContact(Arms::right);
//         CR_YIELD();
//       }
//     }
// 
//   private:
//     DEFINES_PARAMS(ArmContactAnyArmTask,
//     {,
//       (int)(6000) stayBackTime, /**< The duration for which an arm stays back after contact. */
//       (int)(2000) deadTime, /**< The duration for which new contacts are ignored after the arm has been back. */
//     });
// 
//     READS(FrameInfo);
//     READS(ArmContactModel);
// 
//     ArmSkills armSkills  {env};
// 
//     unsigned timesOfLastContact[Arms::numOfArms]; /**< The last time when a contact has been detected that should be reacted upon. */
//     unsigned timesOfLastAction[Arms::numOfArms]; /**< The last time when an arm was taken back by this skill. */
// 
// 
//     void moveArmIfContact(Arms::Arm arm)
//     {
//       if (SystemCall::getMode() == SystemCall::simulatedRobot)
//         return;
//       if (theArmContactModel.status[arm].contact &&
//           (theArmContactModel.status[arm].pushDirection == ArmContactModel::backward ||
//            theArmContactModel.status[arm].pushDirection == ArmContactModel::left ||
//            theArmContactModel.status[arm].pushDirection == ArmContactModel::right) &&
//           theFrameInfo.getTimeSince(timesOfLastAction[arm]) >= params.deadTime)
//         timesOfLastContact[arm] = theFrameInfo.time;
// 
//       if (theFrameInfo.getTimeSince(timesOfLastContact[arm]) < params.stayBackTime)
//       {
//         armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm);
//         timesOfLastAction[arm] = theFrameInfo.time;
//       }
//     }
//   };
// 
// 
// 
//   /**
//    * Reacts to nearby obstacles by taking the specified arm away.
//    */
//   CRBEHAVIOUR(ArmObstacleAvoidanceTask)
//   {
//     CRBEHAVIOUR_INIT(ArmObstacleAvoidanceTask) {}
// 
//     /**
//      * Reacts to nearby obstacles by taking the specified arm away.
//      * @param whichArm The arm for which the obstacle detection/reaction shall be done
//      */
//     void operator()(Arms::Arm whichArm)
//     {
//       CRBEHAVIOUR_BEGIN();
// 
//       arm = whichArm; // the arm can only be changed when the task has been reset
// 
//       while (true)
//       {
//         CR_CHECKPOINT(noAvoidanceNeeded);
//         CR_AWAIT(shouldDoAvoidance(arm), CR_NOP());
// 
//         CR_CHECKPOINT(doAvoidance);
//         CR_WHILE(shouldDoAvoidance(arm), armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm));
// 
//         CR_CHECKPOINT(finishAvoidance);
//         CR_WHILE(getCheckpointDuration() < params.finishTime, armSkills.keyFrameArm(ArmKeyFrameRequest::back, arm));
// 
//         CR_CHECKPOINT(deadTime);
//         CR_WHILE(getCheckpointDuration() < params.deadTime, CR_NOP());
//       }
//     }
// 
//   private:
//     DEFINES_PARAMS(ArmObstacleAvoidanceTask,
//     {,
//       (unsigned)(1000) finishTime,  /**< [ms] The duration for which an arm stays back after obstacle no longer detected. */
//       (unsigned)(500) deadTime,     /**< [ms] The duration for which new contacts are ignored after the arm has been back. */
//       (float)(400.f) distance, /**< [mm] Only obstacles closer than this are considered. */
//     });
// 
//     READS(FrameInfo);
//     READS(ObstacleModel);
//     READS(MotionInfo);
//     READS(FallDownState);
// 
//     ArmSkills armSkills  {env};
// 
//     Arms::Arm arm; // the arm to use
// 
//     bool shouldDoAvoidance(Arms::Arm arm) const
//     {
//       if (theMotionInfo.executedPhase != MotionPhase::walk || theFallDownState.state != FallDownState::upright)
//         return false;
// 
//       const float distanceSquared = sqr(params.distance);
//       // This sign makes the inequality in which it is used applicable for left as well as right.
//       const float sign = arm == Arms::left ? 1.f : -1.f;
//       for(const Obstacle& obstacle : theObstacleModel.obstacles)
//         if((obstacle.left.squaredNorm() < distanceSquared && sign * obstacle.left.angle() > pi_8) ||
//           (obstacle.center.squaredNorm() < distanceSquared && sign * obstacle.center.angle() > pi_8) ||
//           (obstacle.right.squaredNorm() < distanceSquared && sign * obstacle.right.angle() > pi_8))
//           return true;
// 
//       return false;
//     }
//   };
// 
// 
//   /**
//    * Reacts to nearby obstacles on the right/left by taking the appropriate arm away.
//    */
//   CRBEHAVIOUR(ArmObstacleAvoidanceAnyArmTask)
//   {
//     CRBEHAVIOUR_INIT(ArmObstacleAvoidanceAnyArmTask) {}
// 
//     /**
//      * Reacts to nearby obstacles on the right/left by taking the appropriate arm away.
//      */
//     void operator()()
//     {
//       CR_LOOP()
//       {
//         leftArmObstacleAvoidanceTask(Arms::left);
//         rightArmObstacleAvoidanceTask(Arms::right);
//         CR_YIELD();
//       }
//     }
// 
//   private:
//     ArmObstacleAvoidanceTask leftArmObstacleAvoidanceTask {env};
//     ArmObstacleAvoidanceTask rightArmObstacleAvoidanceTask {env};
//   };
// } // namespace RE




