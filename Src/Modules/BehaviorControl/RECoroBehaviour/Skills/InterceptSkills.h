/**
 * @file InterceptSkills.h
 *
 * Intercept behaviour for outfield players.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BallSkills.h"

#include "Tools/BehaviorControl/Interception.h"


namespace RE
{
  struct IgnoreMe {}; // FIXME - either need to rename the CoroBehaviour namespace or the CoroBehaviour class throughtout codebase

  /**
   * Intercept skills are focused on intercepting a ball that will pass the 
   * robot in the next few seconds (E.g. 5 secs). If the ball will not pass
   * for a while or is too far away, the robot walks to intercept. Otherwise 
   * the robot does some intercept move.
   * 
   * Generally if the ball will take even longer to pass the robot or is
   * too far away we need a higher level behaviour to take over.
   */

  // struct InterceptSkills
  // {
  //   InterceptSkills(BehaviourEnv& env) : env(env) {}
// 
//     // ------------------------------------------------------------------------
//     // stateless functions to call directly (without declaring as a task)
//     // ------------------------------------------------------------------------
// 
//     bool isInterceptNeeded()
//     {
//       if (!theFieldBall.ballWasSeen(params.ballSeenInterceptTimeoutMs))
//         return false;
//       
//       float interceptY = std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y());
//       float interceptYSecs = theFieldInterceptBall.timeUntilIntersectsOwnYAxis;
// 
//       float interceptX = std::fabs(theFieldBall.intersectionPositionWithOwnXAxis.x());
//       float interceptXSecs = theFieldBall.secsUntilIntersectsOwnXAxis;
// 
//       // intercept needed if the ball will intercept robot y-axis within allowed time range
//       //      AND interceptY position is reachable by keyframe motion or walking
//       // OR the ball will intercept robot x-axis within allowed time range
//       //      AND interceptX position is reachable by walking
//       return (params.interceptYSecsRange.isInside(interceptYSecs) &&
//              (interceptY < std::max(params.keyframeInterceptY, params.walkInterceptSpeed * interceptYSecs))) ||
//              ((params.interceptXSecsRange.isInside(interceptXSecs) &&
//              (interceptX < (params.walkInterceptSpeed * interceptXSecs))));
//     }
// 
//   private:
//     struct // note: not a CRBEHAVIOUR so has to use this struct instead of DEFINES_PARAMS 
//     {
//       unsigned ballSeenInterceptTimeoutMs = 500; // we can only intercept the ball if seen very recently
//       float keyframeInterceptY = 230.f; ///< max y at which keyframe intercepts make sense (see genuflect stuff params in InterceptTask)
//       float walkInterceptSpeed = 200.f; ///< mm/sec - conservative to allow some time for alignment
//       Rangef interceptYSecsRange {0.1f, 7.f}; ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
//       Rangef interceptXSecsRange {0.1f, 7.f}; ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
//     }
//     params;
// 
//     BehaviourEnv& env;
// 
//     READS(FieldBall);
//   };

  // =======================================================================

  /**
   * Perform a keyframed intercept motion
   */
  CRBEHAVIOUR(KeyFrameInterceptTask)
  {
    CRBEHAVIOUR_INIT(KeyFrameInterceptTask) {}

    void operator()(MotionRequest::Dive::Request interceptMotionIn, unsigned getUpDelayIn = 0)
    {
      CRBEHAVIOUR_BEGIN();

      latchParams(interceptMotionIn, getUpDelayIn);

      // execute selected intercept motion

      CR_CHECKPOINT(intercept);
      actionDone = false;
      while (!actionDone)
      {
        headSkills.lookForward();
        actionDone = motionSkills.dive(interceptMotion);
        CR_YIELD();
      }

      // wait for intercept motion to complete

      CR_CHECKPOINT(wait_for_intercept_to_complete);
      while (getCoroDuration() < waitForInterceptMs) // time since start of coro, not just checkpoint
      {
        headSkills.lookForward();
        motionSkills.dive(interceptMotion);
        CR_YIELD();
      }

      // wait for additional get up delay if needed

      CR_CHECKPOINT(wait_for_getup_delay);
      while (getCheckpointDuration() < getUpDelay)
      {
        headSkills.lookForward();
        motionSkills.dive(interceptMotion);
        CR_YIELD();
      }

      // finally, execute the get up

      CR_CHECKPOINT(get_up_after_intercept);
      while (true)
      {
        headSkills.lookForward();
        // if (motionSkills.getUp())
        if (motionSkills.stand())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(KeyFrameInterceptTask, 
    {,
      (unsigned)(2000) waitForGenuflectMs,
      // (unsigned)(300) ballSeenTimeoutMs, ///< we can only save the ball if we've seen it very recently
      // (Rangef)(0.05f, 3.f) interceptTimeRangeSecs, ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
    });

    READS(FieldBall);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    bool actionDone;
    MotionRequest::Dive::Request interceptMotion;
    unsigned getUpDelay;
    // bool canSeeBallAfterIntercept;
    unsigned waitForInterceptMs;

    void latchParams(MotionRequest::Dive::Request interceptMotionIn, unsigned getUpDelayIn)
    {
      interceptMotion = interceptMotionIn;
      getUpDelay = getUpDelayIn;

      switch (interceptMotion)
      {
      case MotionRequest::Dive::squatArmsBackLeft:
      case MotionRequest::Dive::squatArmsBackRight:
      case MotionRequest::Dive::squatWideArmsBackLeft:
      case MotionRequest::Dive::squatWideArmsBackRight:
        waitForInterceptMs = params.waitForGenuflectMs;
        break;
      default:
        waitForInterceptMs = 0;
      }
    }
  };


  // =======================================================================

  CRBEHAVIOUR(InterceptTask)
  {
    CRBEHAVIOUR_INIT(InterceptTask) {}

    void operator()(unsigned interceptsEnabled, bool aggressive = false, unsigned getUpDelay = 0)
    {
      CRBEHAVIOUR_BEGIN();

      // if we already called isInterceptNeeded in this frame then wasInterceptNeeded
      // has already been changed if required
      if (timeWhenInterceptNeededChanged != theFrameInfo.time) 
        wasInterceptNeeded = false;

      while (isInterceptNeeded(aggressive))
      {
        // decide which save to make
        chooseInterceptMotion(interceptsEnabled);
        
        if (interceptMode == STAY_AS_IS) // no intercept selected?
        {
          CR_CHECKPOINT(stay_as_is);
          headSkills.lookAtBall();
          motionSkills.stand();

          CR_YIELD();
          // CR_EXIT_SUCCESS();
        }
        else if (interceptMode == WALK)
        {
          CR_CHECKPOINT(walk_intercept);
          headSkills.lookAtBall();
          walkToPoseTask(getInterceptPose(), /*speed*/ 1.f, AVOIDANCE_ROUGH,
                          /*keepTargetRotation*/ true, /*standAtTarget*/ false);

          CR_YIELD();          
        }
        else // KEYFRAME
        {
          CR_CHECKPOINT(keyframe_intercept);
          CR_AWAIT(keyFrameInterceptTask.isSuccess(), keyFrameInterceptTask(interceptMotion, getUpDelay));
        }
      }

      TLOGD(tlogger, "interceptTask intercept no longer needed");

      // intercept no longer needed
      CR_CHECKPOINT(intercept_no_longer_needed);
      headSkills.lookAtBall();
      if (motionSkills.isWalking())
        motionSkills.walkAtRelativeSpeed(Pose2f(0,0,0));
      else
        motionSkills.stand();
      CR_EXIT_SUCCESS();
    }

    bool isInterceptNeeded(bool aggressive = false)
    {
      bool needed = true;

      // check for shortcut answer depending on whether we needed an
      // intercept in the last frame or not
      if (wasInterceptNeeded) // was an intercept needed during the last frame
      {
        if (!theFieldBall.ballWasSeen(params.ballSeenTimeoutMs) ||
            (theFieldBall.timeSinceBallDisappeared > params.ballDisappearedTimeoutMs))
        {
          needed = false;
          TLOGD(tlogger, "isInterceptNeeded: ball timeout seen {} disappeared {}", theFieldBall.timeSinceBallWasSeen,
                theFieldBall.timeSinceBallDisappeared);
        }
      }
      else // no intercept needed last frame
      {
        if (!theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
        {
          needed = false;
          TLOGV(tlogger, "isInterceptNeeded: ball timeout seen {}", theFieldBall.timeSinceBallWasSeen);
        }
      }

      updateInterceptValues(aggressive);

      if (needed) // now check if really needed
        needed = shouldInterceptY || shouldInterceptX;
      else
        TLOGV(tlogger, "isInterceptNeeded: shouldInterceptY {}, shouldInterceptX {}", shouldInterceptY,
              shouldInterceptX);

      if (needed != wasInterceptNeeded) // did we change from needed to not-needed or vice versa?
      {
        const char *neededStr = needed ? "yes-start" : "no-finish";
        const char *aggrStr = isAggressiveY ? " (aggr)" : "";

        CRB_ANNOTATION_FMT("isInterceptNeeded {}: y={:.0f} ySecs={:.1f}{}, x={:.0f} xSecs={:.1f}", neededStr,
                           interceptY, std::min(interceptYSecs, 999.f), aggrStr, interceptX,
                           std::min(interceptXSecs, 999.f));
        TLOGV(tlogger, "isInterceptNeeded {}: y={:.0f} ySecs={:.1f}{}, x={:.0f} xSecs={:.1f}", neededStr, interceptY,
              std::min(interceptYSecs, 999.f), aggrStr, interceptX, std::min(interceptXSecs, 999.f));

        wasInterceptNeeded = needed;
        timeWhenInterceptNeededChanged = theFrameInfo.time;
      }

      return needed;
    }


    static constexpr unsigned DEFENDER_INTERCEPTS = bit(Interception::genuflectStandDefender) | bit(Interception::walk);
    static constexpr unsigned OUTFIELD_INTERCEPTS = bit(Interception::genuflectStand) | bit(Interception::walk);
    static constexpr unsigned WALK_INTERCEPTS = bit(Interception::walk);

  private:
    DEFINES_PARAMS(InterceptTask, 
    {,
      (Rangef)(0.f, 30.f) standRange, ///< a ball inside this y-distance can be intercepted by just standing
      (Rangef)(30.f, 230.f) genuflectRange, ///< a ball inside this y-distance can be intercepted using the genuflect motion
      (unsigned)(2000) waitForGenuflectMs,
      // (unsigned)(300) ballSeenTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(0.1f, 3.f) keyframeTimeRangeSecs, ///< we don't try a keyframe intercept if the ball will pass too quick or too far in the future
      (float)(100.f) walkXThreshold, ///< walk to intercept if the x-intercept is less than this and would hit the side of the feet
      (Angle)(135_deg) sideApproachAngle, ///< consider angles less than this to be side approach (we don't expect angles < 90)

      (unsigned)(1000) ballSeenTimeoutMs, ///< we can only intercept the ball if seen pretty recently
      (int)(500) ballDisappearedTimeoutMs, ///< ball disappeared means it should have been visible in the image but wasn't
      (float)(230.f) keyframeInterceptY, ///< max y at which keyframe intercepts make sense (see genuflect stuff params in InterceptTask)
      (float)(240.f) walkInterceptSpeedY, ///< mm/sec - ambitious so we give it a shot, even if we might not make it
      (float)(180.f) walkInterceptSpeedX, ///< mm/sec - conservative so we don't try for x intercept generally
      (Rangef)(0.05f, 8.f) interceptYSecsRange, ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future
      (Rangef)(0.05f, 4.f) interceptXSecsRange, ///< we don't try to intercept if the ball will pass too quick or not save yet if too far in the future

    });

    READS(FieldBall);
    READS(FieldInterceptBall);
    READS(BallModel);
    READS(RobotDimensions);
    READS(FrameInfo);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    WalkToPoseTask walkToPoseTask {env};
    KeyFrameInterceptTask keyFrameInterceptTask {env};

    DECL_TLOGGER_VAR(tlogger, "InterceptTask", TextLogging::DEBUG);

    // bool actionDone;
    enum { STAY_AS_IS, WALK, KEYFRAME } interceptMode;
    MotionRequest::Dive::Request interceptMotion;
    bool canSeeBallAfterIntercept;
    bool wasInterceptNeeded;
    unsigned timeWhenInterceptNeededChanged = 0;

    float interceptY;
    float interceptYSecs;
    bool shouldInterceptY; // should a y intercept be performed
    bool isAggressiveY; // is a y intercept only to be performed according if aggressively anticipate interceptions
    float interceptX;
    float interceptXSecs;
    bool shouldInterceptX;


    void updateInterceptValues(bool aggressive)
    {
      shouldInterceptY = false;
      isAggressiveY = false;
      shouldInterceptX = false;

      // y-intercept first

      interceptY = theFieldInterceptBall.intersectionPositionWithOwnYAxis.y();
      interceptYSecs = theFieldInterceptBall.timeUntilIntersectsOwnYAxis;

      // TODO - not implemented yet for BH2025 framework
      // calculate more aggressive intercept if needed
      // if (aggressive && (interceptYSecs == std::numeric_limits<float>::max()))
      // {
      //   interceptY = theFieldInterceptBall.intersectionPositionWithOwnYAxisAggressive.y();
      //   interceptYSecs = theFieldBall.secsUntilIntersectsOwnYAxisAggressive;
      //   isAggressiveY = true;
      // }

      // is y-intercept feasible/needed

      // FIXME - this logic should also consider whether keyframe intercepts are allowed
      shouldInterceptY = params.interceptYSecsRange.isInside(interceptYSecs);
      if (shouldInterceptY && !aggressive)
        shouldInterceptY =
            (std::abs(interceptY) < std::max(params.keyframeInterceptY, params.walkInterceptSpeedY * interceptYSecs));

      interceptX = theFieldInterceptBall.intersectionPositionWithOwnXAxis.x();
      interceptXSecs = theFieldInterceptBall.timeUntilIntersectsOwnXAxis;

      // is x-intercept feasible/recommended
      // the ball will intercept robot x-axis within allowed time range
      // AND interceptX position is reachable by walking
      shouldInterceptX = !shouldInterceptY && params.interceptXSecsRange.isInside(interceptXSecs);
      if (shouldInterceptX && !aggressive)
        shouldInterceptY = (std::abs(interceptX) < (params.walkInterceptSpeedX * interceptXSecs));
    }


    Pose2f getInterceptPose()
    {
      // if (theFieldInterceptBall.timeUntilIntersectsOwnYAxis < theFieldBall.secsUntilIntersectsOwnXAxis)
      if (shouldInterceptY)
      {
        return Pose2f(0.f, 0.f, interceptY);
      }
      else if (shouldInterceptX)
      {
        // the ball will intercept our x-pos so we should turn partly to face it
        Angle ballAngle = theFieldBall.positionRelative.angle();
        return Pose2f(ballAngle, interceptX - (theRobotDimensions.footLength + 50.f), 0.f);
      }
      else
      {
        return Pose2f(0,0,0);
      }
    }

    void chooseInterceptMotion(unsigned interceptsEnabled)
    {
      // defaults
      interceptMode = KEYFRAME;
      canSeeBallAfterIntercept = true;

      bool left = (interceptY >= 0);
      float interceptYDistance = std::fabs(interceptY);
      bool insideKeyframeTimeRange = params.keyframeTimeRangeSecs.isInside(interceptYSecs);

      // bool canIntercept =
      //     (theFieldBall.ballWasSeen(300) && between<float>(theFieldInterceptBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f));
      // bool noReactionNeeded = (interceptTime > 10.f) || (interceptDistance > std::max(230.f, interceptTime * 150.f));

      if (params.standRange.isInside(interceptYDistance))
      {
        // if ball is approaching from the side and will hit the side of the feet we want to walk to position
        if ((interceptX < params.walkXThreshold) &&
            (std::abs(theBallModel.estimate.velocity.angle()) < params.sideApproachAngle))
          interceptMode = WALK;
        else
          interceptMode = STAY_AS_IS; // continue to stand/sit
      }
      else if (!isAggressiveY && insideKeyframeTimeRange && (interceptsEnabled & bit(Interception::genuflectStand)) &&
               params.genuflectRange.isInside(interceptYDistance))
        // interceptMotion = KeyframeMotionRequest::genuflectStand; // sumo wide stance
        interceptMotion = left ? MotionRequest::Dive::squatArmsBackLeft : MotionRequest::Dive::squatArmsBackRight;
      else if (!isAggressiveY && insideKeyframeTimeRange &&
               (interceptsEnabled & bit(Interception::genuflectStandDefender)) &&
               params.genuflectRange.isInside(interceptYDistance))
        // interceptMotion = KeyframeMotionRequest::genuflectStandDefender; // sumo wide stance
        interceptMotion = left ? MotionRequest::Dive::squatWideArmsBackLeft : MotionRequest::Dive::squatWideArmsBackRight;
      else // walk
        interceptMode = WALK;
    }
  };

} // namespace RE