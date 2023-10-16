/**
 * @file: CommonSkills.h
 *
 * Common, useful, or combined skills that are called often enough to deserve
 * a shorter version
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BehaviorStatusSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/SoundSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"


namespace CoroBehaviour
{

  struct CommonSkills
  {
    CommonSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions to call directly (including some repeats from
    // HeadSkills, MotionSkills, BehaviorStatusSkills for convenience)
    // ------------------------------------------------------------------------

    // from BehaviorStatusSkills

    inline void activityStatus(BehaviorStatus::Activity activity) { statusSkills.activity(activity); }

    inline void passTargetStatus(int passTarget, const Vector2f& ballTarget = Vector2f::Zero())
    {
      statusSkills.passTarget(passTarget, ballTarget);
    }

    inline void walkingToStatus(const Vector2f& target, float speed = 1.f) { statusSkills.walkingTo(target, speed); }

    // from SoundSkills

    inline void say(const std::string &text) { soundSkills.say(text); }

    inline void playSound(const std::string &name) { soundSkills.playSound(name); }

    // from HeadSkills

    inline void lookForward() { headSkills.lookForward(); }

    inline void lookDown() { headSkills.lookDown(); }

    inline void lookActive() { headSkills.lookActive(); }

    // from MotionSkills

    inline bool unstiff() { return motionSkills.unstiff(); }

    inline bool stand(bool high = false) { return motionSkills.stand(high); }


    // combined head and motion skills

    /**
     * stand and look straight ahead (e.g. in robocup initial state)
     */
    void standLookForward(bool standHigh = false)
    {
      headSkills.lookForward();
      motionSkills.stand(standHigh);
    }

    /**
     * stand and look down, e.g. when the robot is penalized
     */
    void standLookDown(bool standHigh = false) 
    { 
      headSkills.lookDown();
      motionSkills.stand(standHigh);
    }

    /**
     * stand and look active
     */
    void standLookActive(bool standHigh = false)
    {
      headSkills.lookActive();
      motionSkills.stand(standHigh);
    }


    // game functions

    bool isOurTeamKick() { return theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber; }

    // coordinate systems

    // /**
    //  * convert from field to robot relative coordinates
    //  */
    // Vector2f toRobotCoordinates(const Vector2f &fieldPoint) { return theRobotPose.inversePose * fieldPoint; }
    // Pose2f toRobotCoordinates(const Pose2f &fieldPose) { return theRobotPose.inversePose * fieldPose; }

    // /**
    //  * convert from robot relative to field coordinates
    //  */
    // Vector2f toFieldCoordinates(const Vector2f &relativePoint) { return theRobotPose * relativePoint; }
    // Pose2f toFieldCoordinates(const Pose2f &relativePose) { return theRobotPose * relativePose; }


    /**
     * check if a relative pose is within specified thresholds
     */
    static bool isPoseClose(const Pose2f& pose, float distanceThreshold = 100.f, Angle angleThreshold = 5_deg)
    {
      return (pose.translation.squaredNorm() < sqr(distanceThreshold)) && (fabs(pose.rotation) < angleThreshold);
    }

    /**
     * check if a vector2f is within specified thresholds
     */
    static bool isPointClose(const Vector2f& pos, float distanceThreshold = 100.f)
    {
      return (pos.squaredNorm() < sqr(distanceThreshold));
    }

  private:
    BehaviourEnv& env;

    READS(RobotPose);
    READS(GameInfo);
    READS(OwnTeamInfo);

    BehaviorStatusSkills statusSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    SoundSkills soundSkills;
  };



  /// placeholder task that does nothing much - useful during behaviour development
  /// it doesn't specify an activity
  CRBEHAVIOUR(StandLookForwardTask)
  {
    CRBEHAVIOUR_INIT(StandLookForwardTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        commonSkills.activityStatus(BehaviorStatus::reStandStill);
        commonSkills.standLookForward(true); // high
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills  {env};
  };

  /// placeholder task that just stands and also sets the activity to placeholder
  CRBEHAVIOUR(PlaceholderTask)
  {
    PlaceholderTask(BehaviourEnv& env, const char * name="PlaceholderTask") : CoroBehaviour(env, name) {}
    // CRBEHAVIOUR_INIT(PlaceholderTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        commonSkills.activityStatus(BehaviorStatus::rePlaceholder);
        commonSkills.standLookForward(true); // high
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills  {env};
  };


  /// placeholder task that just stands and also sets the activity to placeholder and succeeds after a short time
  CRBEHAVIOUR(PlaceholderSuccessTask)
  {
    PlaceholderSuccessTask(BehaviourEnv& env, const char * name="PlaceholderSuccessTask") : CoroBehaviour(env, name) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      while (true)
      {
        commonSkills.activityStatus(BehaviorStatus::rePlaceholder);
        commonSkills.standLookForward(true); // high
        if (getCoroDuration() > SUCCESS_DURATION_MS)
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    const unsigned SUCCESS_DURATION_MS = 500;

    CommonSkills commonSkills  {env};
  };


} // CoroBehaviour
