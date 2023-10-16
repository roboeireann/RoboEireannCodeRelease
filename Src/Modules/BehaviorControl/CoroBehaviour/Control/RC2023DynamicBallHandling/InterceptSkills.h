/**
 * @file InterceptSkills.h
 *
 * Intercept behaviour for outfield players.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Tools/BehaviorControl/Interception.h"


namespace CoroBehaviour
{
namespace RC2023
{
  /**
   * Intercept skills are focused on intercepting a ball that will pass the 
   * robot in the next few seconds (E.g. 5 secs). If the ball will not pass
   * for a while or is too far away, the robot walks to intercept. Otherwise 
   * the robot does some intercept move.
   * 
   * Generally if the ball will take even longer to pass the robot or is
   * too far away we need a higher level behaviour to take over.
   */

  struct InterceptSkills
  {
    InterceptSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions to call directly (without declaring as a task)
    // ------------------------------------------------------------------------

    bool isInterceptNeeded()
    {
      if (!theFieldBall.ballWasSeen(params.ballSeenInterceptTimeoutMs))
        return false;
      
      float interceptY = std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y());
      float interceptYSecs = theFieldBall.timeUntilIntersectsOwnYAxis;

      float interceptX = std::fabs(theFieldBall.intersectionPositionWithOwnXAxis.x());
      float interceptXSecs = theFieldBall.timeUntilIntersectsOwnXAxis;

      return (params.interceptYSecsRange.isInside(interceptYSecs) &&
             (interceptY < std::max(params.keyframeInterceptY, params.walkInterceptSpeed * interceptYSecs))) ||
             ((params.interceptXSecsRange.isInside(interceptXSecs) &&
             (interceptX < (params.walkInterceptSpeed * interceptXSecs))));
    }

  private:
    struct 
    {
      unsigned ballSeenInterceptTimeoutMs = 500; // we can only intercept the ball if seen very recently
      float keyframeInterceptY = 230.f; ///< max y at which keyframe intercepts make sense (see genuflect stuff params in InterceptTask)
      float walkInterceptSpeed = 200.f; ///< mm/sec - conservative to allow some time for alignment
      Rangef interceptYSecsRange {0.1f, 7.f}; ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
      Rangef interceptXSecsRange {0.1f, 7.f}; ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
    }
    params;

    BehaviourEnv& env;

    READS(FieldBall);
  };

  // =======================================================================

  /**
   * Perform a keyframed intercept motion
   */
  CRBEHAVIOUR(KeyFrameInterceptTask)
  {
    CRBEHAVIOUR_INIT(KeyFrameInterceptTask) {}

    void operator()(KeyframeMotionRequest::KeyframeMotionID interceptMotionIn, unsigned getUpDelayIn = 0)
    {
      CRBEHAVIOUR_BEGIN();

      latchParams(interceptMotionIn, getUpDelayIn);

      // execute selected intercept motion

      CR_CHECKPOINT(intercept);
      actionDone = false;
      while (!actionDone)
      {
        headSkills.lookForward();
        actionDone = motionSkills.keyframeMotion(interceptMotion);
        CR_YIELD();
      }

      // wait for intercept motion to complete

      CR_CHECKPOINT(wait_for_intercept_to_complete);
      while (getCoroDuration() < waitForInterceptMs) // time since start of coro, not just checkpoint
      {
        headSkills.lookForward();
        motionSkills.keyframeMotion(interceptMotion);
        CR_YIELD();
      }

      // wait for additional get up delay if needed

      CR_CHECKPOINT(wait_for_getup_delay);
      while (getCheckpointDuration() < getUpDelay)
      {
        headSkills.lookForward();
        motionSkills.keyframeMotion(interceptMotion);
        CR_YIELD();
      }

      // finally, execute the get up

      CR_CHECKPOINT(get_up_after_intercept);
      while (true)
      {
        headSkills.lookForward();
        if (motionSkills.getUp())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(KeyFrameInterceptTask, 
    {,
      (unsigned)(2000) waitForGenuflectMs,
      (unsigned)(300) ballSeenTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(0.05f, 3.f) interceptTimeRangeSecs, ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
    });

    READS(FieldBall);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    bool actionDone;
    KeyframeMotionRequest::KeyframeMotionID interceptMotion;
    unsigned getUpDelay;
    // bool canSeeBallAfterIntercept;
    unsigned waitForInterceptMs;

    void latchParams(KeyframeMotionRequest::KeyframeMotionID interceptMotionIn, unsigned getUpDelayIn)
    {
      interceptMotion = interceptMotionIn;
      getUpDelay = getUpDelayIn;

      switch (interceptMotion)
      {
        case KeyframeMotionRequest::genuflectStand: waitForInterceptMs = params.waitForGenuflectMs; break;
        case KeyframeMotionRequest::genuflectStandDefender: waitForInterceptMs = params.waitForGenuflectMs; break;
        default: waitForInterceptMs = 0;
      }
    }
  };


  // =======================================================================

  CRBEHAVIOUR(InterceptTask)
  {
    CRBEHAVIOUR_INIT(InterceptTask) {}

    void operator()(unsigned interceptsEnabled, unsigned getUpDelay = 0)
    {
      CRBEHAVIOUR_LOOP()
      {
        // decide which save to make
        chooseInterceptMotion(interceptsEnabled);
        
        if (interceptMode == STAY_AS_IS) // no intercept selected?
        {
          CR_CHECKPOINT(stay_as_is);
          headSkills.lookAtBall();
          motionSkills.stand();

          CR_EXIT_SUCCESS();
        }
        else if (interceptMode == WALK)
        {
          CR_CHECKPOINT(walk_intercept);
          headSkills.lookAtBall();
          // FIXME - switch to coro version
          CALL_SKILL(WalkToPoint)
              (getInterceptPose(), 1.f, /* rough: */ false,
              /* disableObstacleAvoidance: */ false, /* disableAligning: */ true);

          CR_YIELD();
          
        }
        else // KEYFRAME
        {
          CR_CHECKPOINT(keyframe_intercept);
          CR_AWAIT(keyFrameInterceptTask.isSuccess(), keyFrameInterceptTask(interceptMotion, getUpDelay));
          CR_EXIT_SUCCESS();
        }
      }
    }

    static constexpr unsigned DEFENDER_INTERCEPTS = bit(Interception::genuflectStandDefender) | bit(Interception::walk);
    static constexpr unsigned OUTFIELD_INTERCEPTS = bit(Interception::genuflectStand) | bit(Interception::walk);
    static constexpr unsigned WALK_INTERCEPTS = bit(Interception::walk);

  private:
    DEFINES_PARAMS(InterceptTask, 
    {,
      (Rangef)(0.f, 80.f) standRange, ///< a ball inside this y-distance can be intercepted by just standing
      (Rangef)(80.f, 230.f) genuflectRange, ///< a ball inside this y-distance can be intercepted using the genuflect motion
      (unsigned)(2000) waitForGenuflectMs,
      (unsigned)(300) ballSeenTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(0.1f, 3.f) keyframeTimeRangeSecs, ///< we don't try a keyframe intercept if the ball will pass too quick or too far in the future
      (float)(100.f) walkXThreshold, ///< walk to intercept if the x-intercept is less than this and would hit the side of the feet
      (Angle)(135_deg) sideApproachAngle, ///< consider angles less than this to be side approach (we don't expect angles < 90)
    });

    READS(FieldBall);
    READS(BallModel);
    READS(RobotDimensions);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    KeyFrameInterceptTask keyFrameInterceptTask {env};

    // bool actionDone;
    enum { STAY_AS_IS, WALK, KEYFRAME } interceptMode;
    KeyframeMotionRequest::KeyframeMotionID interceptMotion;
    bool canSeeBallAfterIntercept;


    Pose2f getInterceptPose()
    {
      if (theFieldBall.timeUntilIntersectsOwnYAxis < theFieldBall.timeUntilIntersectsOwnXAxis)
        return Pose2f(0.f, 0.f, theFieldBall.intersectionPositionWithOwnYAxis.y());
      else
        return Pose2f(0.f, theFieldBall.intersectionPositionWithOwnXAxis.x() - (theRobotDimensions.footLength + 100.f), 0.f);
    }

    void chooseInterceptMotion(unsigned interceptsEnabled)
    {
      // defaults
      interceptMode = KEYFRAME;
      canSeeBallAfterIntercept = true;

      float interceptYDistance = std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y());
      float interceptYSecs = theFieldBall.timeUntilIntersectsOwnYAxis;
      bool insideKeyframeTimeRange = params.keyframeTimeRangeSecs.isInside(interceptYSecs);

      // bool canIntercept =
      //     (theFieldBall.ballWasSeen(300) && between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f));
      // bool noReactionNeeded = (interceptTime > 10.f) || (interceptDistance > std::max(230.f, interceptTime * 150.f));

      if (params.standRange.isInside(interceptYDistance))
      {
        // if ball is approaching from the side and will hit the side of the feet we want to walk to position
        if ((theFieldBall.intersectionPositionWithOwnXAxis.x() < params.walkXThreshold) &&
            (std::abs(theBallModel.estimate.velocity.angle()) < params.sideApproachAngle))
          interceptMode = WALK;
        else
          interceptMode = STAY_AS_IS; // continue to stand/sit
      }
      else if (insideKeyframeTimeRange && (interceptsEnabled & bit(Interception::genuflectStand)) &&
               params.genuflectRange.isInside(interceptYDistance))
        interceptMotion = KeyframeMotionRequest::genuflectStand; // sumo wide stance
      else if (insideKeyframeTimeRange && (interceptsEnabled & bit(Interception::genuflectStandDefender)) &&
               params.genuflectRange.isInside(interceptYDistance))
        interceptMotion = KeyframeMotionRequest::genuflectStandDefender; // sumo wide stance
      else // walk
        interceptMode = WALK;
    }
  };

} // RC2023
} // CoroBehaviour2023