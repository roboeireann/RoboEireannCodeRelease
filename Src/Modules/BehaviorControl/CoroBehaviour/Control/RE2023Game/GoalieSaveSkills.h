/**
 * @file GoalieSaveSkills.h
 *
 * This file contains some RE2019 based goalie save skills.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
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
namespace RE2023
{
  // =======================================================================

  CRBEHAVIOUR(GoalieSaveTask)
  {
    CRBEHAVIOUR_INIT(GoalieSaveTask) {}

    void operator()(unsigned savesEnabled, bool isSitting = false, unsigned getUpDelay = 0)
    {
      CRBEHAVIOUR_BEGIN();

      // disable saves that don't make sense depending on whether we are sitting or not
      if (isSitting)
        savesEnabled &= ~(bit(Interception::genuflectStand) | bit(Interception::genuflectStandDefender));
      else // standing
        savesEnabled &= ~(bit(Interception::genuflectFromSitting));

      // decide which save to make
      chooseGoalieSaveMotion(savesEnabled);
      
      if (stayAsIs) // no save selected?
      {
        CR_CHECKPOINT(stay_as_is);
        headSkills.lookAtBall();
        if (isSitting)
          motionSkills.keyframeMotion(KeyframeMotionRequest::sitDownKeeper);
        else
          motionSkills.stand();

        CR_EXIT_SUCCESS();
      }

      // Not just staying as-is, so execute selected save motion

      CR_CHECKPOINT(goalie_save);
      actionDone = false;
      while (!actionDone)
      {
        headSkills.lookForward();
        actionDone = motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        CR_YIELD();
      }

      // wait for save to complete

      CR_CHECKPOINT(wait_for_save);
      while (waitForSave())
      {
        headSkills.lookForward();
        motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        CR_YIELD();
      }

      // wait for additional get up delay (so that we try not to knock the ball in
      // while getting up)

      CR_CHECKPOINT(wait_for_getup_delay);
      while (getCheckpointDuration() < getUpDelay)
      {
        headSkills.lookForward();
        motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        CR_YIELD();
      }

      // finally, execute the get up

      CR_CHECKPOINT(get_up_after_save);
      actionDone = false;
      while (!actionDone)
      {
        headSkills.lookForward();
        actionDone = motionSkills.getUp();
        CR_YIELD();
      }

      // stand and look at the ball, indicate success

      CR_CHECKPOINT(save_complete);

      headSkills.lookAtBall();
      motionSkills.stand();
      CR_EXIT_SUCCESS();
    }

  private:
    DEFINES_PARAMS(GoalieSaveTask, 
    {,
      (bool)(true) otherSavesEnabled,
      (bool)(true) jumpsEnabled,
      (Rangef)(0.f, 80.f) standRange,
      (Rangef)(80.f, 150.f) genuflectFromSittingRange,
      (Rangef)(80.f, 230.f) genuflectRange,
      (Rangef)(150.f, 650.f) jumpRange,
      (unsigned)(2000) waitForGenuflectMs,
      (unsigned)(3000) waitForJumpMs,
      (unsigned)(300) ballSeenSaveTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(0.1f, 3.f) saveTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
    });

    READS(FieldBall);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    bool actionDone;
    bool stayAsIs;
    KeyframeMotionRequest::KeyframeMotionID goalieSaveMotion;
    bool mirror;
    bool canSeeBallAfterSave;
    unsigned waitForSaveMs;


    void chooseGoalieSaveMotion(unsigned enabledSaves)
    {
      // defaults
      stayAsIs = false;
      canSeeBallAfterSave = true;
      mirror = false;

      float interceptDistance =
          std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y());

      if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectFromSitting)) &&
          (params.genuflectFromSittingRange.isInside(interceptDistance)))
      {
        goalieSaveMotion = KeyframeMotionRequest::genuflectFromSitting; // sumo wide stance
        waitForSaveMs = params.waitForGenuflectMs;
      }
      else if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectStand)) &&
               (params.genuflectRange.isInside(interceptDistance)))
      {
        goalieSaveMotion = KeyframeMotionRequest::genuflectStand; // sumo wide stance
        waitForSaveMs = params.waitForGenuflectMs;
      }
      else if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectStandDefender)) &&
               (params.genuflectRange.isInside(interceptDistance)))
      {
        goalieSaveMotion = KeyframeMotionRequest::genuflectStandDefender; // sumo wide stance
        waitForSaveMs = params.waitForGenuflectMs;
      }
      else if ((params.jumpsEnabled) && (enabledSaves & (bit(Interception::jumpLeft) | bit(Interception::jumpRight))) &&
               (params.jumpRange.isInside(interceptDistance)))
      {
        goalieSaveMotion = KeyframeMotionRequest::keeperJumpLeft;
        mirror = (theFieldBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // mirror it for right jump
        canSeeBallAfterSave = false;
        waitForSaveMs = params.waitForJumpMs;
      }
      else // continue to stand/sit
      {
        stayAsIs = true;
        waitForSaveMs = 0;
      }
    }

    bool waitForSave()
    {
      bool timeDone = (getCoroDuration() > waitForSaveMs);
      bool saveStillApplies = false;

      if (canSeeBallAfterSave)
      {
        if ((goalieSaveMotion == KeyframeMotionRequest::genuflectFromSitting) ||
            (goalieSaveMotion == KeyframeMotionRequest::genuflectStand) ||
            (goalieSaveMotion == KeyframeMotionRequest::genuflectStandDefender))
        {
          float interceptDistance =
              std::fabs(theFieldBall.intersectionPositionWithOwnYAxis.y());

          bool interceptInRange = false;
          if (goalieSaveMotion == KeyframeMotionRequest::genuflectFromSitting)
            interceptInRange = params.genuflectFromSittingRange.isInside(interceptDistance);
          else
            interceptInRange = params.genuflectRange.isInside(interceptDistance);

          saveStillApplies =
              theFieldBall.ballWasSeen(params.ballSeenSaveTimeoutMs) &&
              interceptInRange &&
              params.saveTimeRangeSecs.isInside(theFieldBall.timeUntilIntersectsOwnYAxis);
        }
      }

      return  !timeDone || saveStillApplies;
    }
  };

} // RE2023
} // CoroBehaviour2023