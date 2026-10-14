/**
 * @file: CommonSkills.h
 *
 * Common, useful, or combined skills that are called often enough to deserve
 * a shorter version
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BehaviorStatusSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/SoundSkills.h"



namespace RE
{

  struct CommonSkills
  {
    CommonSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions to call directly (including some repeats from
    // HeadSkills, MotionSkills, BehaviorStatusSkills for convenience)
    // ------------------------------------------------------------------------

    // from BehaviorStatusSkills

    // inline void recordActivity(BehaviorStatus::Activity activity) { statusSkills.recordActivity(activity); }

    inline void recordPassTarget(int passTarget, const Vector2f& ballTarget = Vector2f::Zero())
    {
      statusSkills.recordPassTarget(passTarget, ballTarget);
    }

    inline void recordWalkingTo(const Vector2f &target, float speed = 1.f)
    {
      statusSkills.recordWalkingTo(target, speed);
    }

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

    bool isOurTeamKick() { return theGameState.isForOwnTeam(); }

    bool isNoTeamKick() { return theGameControllerData.isNoTeamKick(); }

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

    /**
     * check if two points (vector2f) are close to one another
     */
    static bool arePointsClose(const Vector2f& pos1, const Vector2f& pos2, float distanceThreshold = 100.f)
    {
      return (pos2 - pos1).squaredNorm() < sqr(distanceThreshold);
    }

  private:
    BehaviourEnv& env;

    READS(RobotPose);
    READS(GameState);
    READS(GameControllerData);

    BehaviorStatusSkills statusSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    SoundSkills soundSkills;
  };

} // namespace RE
