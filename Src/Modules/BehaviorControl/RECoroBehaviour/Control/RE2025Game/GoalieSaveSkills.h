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

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BallSkills.h"

#include "Tools/BehaviorControl/Interception.h"


namespace RE
{
namespace v2025
{

  // =======================================================================

  // CRBEHAVIOUR(GoalieSaveFromStanding)
  // {
  //   CRBEHAVIOUR_INIT(GoalieSaveFromStanding) {}

  //   void operator()(bool allowGetUp = true)
  //   {
  //     CRBEHAVIOUR_BEGIN();

  //     chooseGoalieSaveMotion();
      
  //     if (justStand) // no save selected?
  //     {
  //       CR_CHECKPOINT(just_stand);
  //       headSkills.lookAtBall();
  //       motionSkills.stand();
  //       CR_EXIT_SUCCESS();
  //     }

  //     // Not just standing, so execute selected save motion

  //     CR_CHECKPOINT(goalie_save);
  //     actionDone = false;
  //     while (!actionDone)
  //     {
  //       headSkills.lookForward();
  //       actionDone = motionSkills.keyframeMotion(goalieSaveMotion, mirror);
  //       CR_YIELD();
  //     }

  //     // wait before recovery and get up (wait indefinitely if get up is not allowed)

  //     CR_CHECKPOINT(wait_for_save);
  //     while (!allowGetUp || waitForSave())
  //     {
  //       headSkills.lookForward();
  //       motionSkills.keyframeMotion(goalieSaveMotion, mirror);
  //       CR_YIELD();
  //     }

  //     // execute the get up

  //     CR_CHECKPOINT(get_up_after_save);
  //     actionDone = false;
  //     while (!actionDone)
  //     {
  //       headSkills.lookForward();
  //       actionDone = motionSkills.getUp();
  //       CR_YIELD();
  //     }

  //     // stand and look at the ball, indicate success

  //     CR_CHECKPOINT(save_complete);
  //     headSkills.lookAtBall();
  //     motionSkills.stand();

  //     CR_EXIT_SUCCESS();
  //   }

  // private:
  //   DEFINES_PARAMS(GoalieSaveFromStanding, 
  //   {,
  //     (bool)(true) otherSavesEnabled,
  //     (bool)(true) jumpsEnabled,
  //     (Rangef)(80.f, 230.f) genuflectStandSaveRange,
  //     (Rangef)(230.f, 600.f) jumpSaveRange,
  //     (unsigned)(2000) waitForGenuflectStandSaveMs,
  //     (unsigned)(3000) waitForJumpSaveMs,
  //     (unsigned)(300) ballSeenSaveTimeoutMs, ///< we can only save the ball if we've seen it very recently
  //     (Rangef)(0.1f, 3.f) saveTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
  //   });

  //   READS(FieldBall);

  //   HeadSkills headSkills {env};
  //   MotionSkills motionSkills {env};

  //   bool actionDone;
  //   bool justStand;
  //   KeyframeMotionRequest::KeyframeMotionID goalieSaveMotion;
  //   bool mirror;
  //   bool canSeeBallAfterSave;
  //   unsigned waitForSaveMs;

  //   // SaveImplTask saveImpl;
  //   // HeadControl::ScanWithBallTask headScanWithBall;
  //   // BallHandling::GotoKickPoseAndKickTask gotoKickPoseAndKick;
  //   // int startMs;

  //   void chooseGoalieSaveMotion()
  //   {
  //     // defaults
  //     justStand = false;
  //     canSeeBallAfterSave = true;
  //     mirror = false;

  //     if (params.otherSavesEnabled &&
  //         params.genuflectStandSaveRange.isInside(std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y())))
  //     {
  //       goalieSaveMotion = KeyframeMotionRequest::genuflectStand; // wide stance
  //       waitForSaveMs = params.waitForGenuflectStandSaveMs;
  //     }
  //     else if (params.jumpsEnabled &&
  //              params.jumpSaveRange.isInside(std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y())))
  //     {
  //       goalieSaveMotion = KeyframeMotionRequest::keeperJumpLeft;
  //       mirror = (theFieldInterceptBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // mirror it for right jump
  //       canSeeBallAfterSave = false;
  //       waitForSaveMs = params.waitForJumpSaveMs;
  //     }
  //     else
  //     {
  //       justStand = true;
  //       waitForSaveMs = 0;
  //     }
  //   }

  //   bool waitForSave()
  //   {
  //     bool timeDone = (getCoroDuration() > waitForSaveMs);
  //     bool saveStillApplies = false;

  //     if (canSeeBallAfterSave)
  //     {
  //       if (goalieSaveMotion == KeyframeMotionRequest::genuflectStand)
  //       {
  //         saveStillApplies =
  //             theFieldBall.ballWasSeen(params.ballSeenSaveTimeoutMs) &&
  //             params.genuflectStandSaveRange.isInside(std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y())) &&
  //             params.saveTimeRangeSecs.isInside(theFieldInterceptBall.timeUntilIntersectsOwnYAxis);
  //       }
  //     }

  //     return  !timeDone && !saveStillApplies;
  //   }
  // };


  // =======================================================================

  CRBEHAVIOUR(GoalieSaveTask)
  {
    CRBEHAVIOUR_INIT(GoalieSaveTask) {}

    void operator()(unsigned enabledSaves, bool isSitting = false, unsigned getUpDelay = 0)
    {
      CRBEHAVIOUR_BEGIN();

      // disable saves that don't make sense depending on whether we are sitting or not
      if (isSitting)
        enabledSaves &= ~(bit(Interception::genuflectStand) | bit(Interception::genuflectStandDefender));
      // else // standing
      //   enabledSaves &= ~(bit(Interception::genuflectFromSitting));

      // decide which save to make
      chooseGoalieSaveMotion(enabledSaves);
      
      if (stayAsIs) // no save selected?
      {
        CR_CHECKPOINT(stay_as_is);
        headSkills.lookAtBall();
        if (isSitting)
          // motionSkills.keyframeMotion(KeyframeMotionRequest::sitDownKeeper);
          motionSkills.dive(MotionRequest::Dive::prepare);
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
        // actionDone = motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        actionDone = motionSkills.dive(goalieDiveMotion);
        CR_YIELD();
      }

      // wait for save to complete

      CR_CHECKPOINT(wait_for_save);
      while (waitForSave())
      {
        headSkills.lookForward();
        // motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        actionDone = motionSkills.dive(goalieDiveMotion);
        CR_YIELD();
      }

      // wait for additional get up delay (so that we try not to knock the ball in
      // while getting up)

      CR_CHECKPOINT(wait_for_getup_delay);
      while (getCheckpointDuration() < getUpDelay)
      {
        headSkills.lookForward();
        // motionSkills.keyframeMotion(goalieSaveMotion, mirror);
        actionDone = motionSkills.dive(goalieDiveMotion);
        CR_YIELD();
      }

      // finally, execute the get up

      CR_CHECKPOINT(get_up_after_save);
      actionDone = false;
      while (!actionDone)
      {
        headSkills.lookForward();
        // actionDone = motionSkills.getUp();
        actionDone = motionSkills.stand();
        CR_YIELD();
      }

      // stand and look at the ball, indicate success

      CR_CHECKPOINT(save_complete);
      // TODO - should we really go back to sitting here after the save or leave it up to the caller?
      // if (isSitting)
      // {
      //   actionDone = false;
      //   while (!actionDone)
      //   {
      //     headSkills.lookAtBall();
      //     motionSkills.keyframeMotion(KeyframeMotionRequest::sitDownKeeper, false);
      //     CR_YIELD();
      //   }
      // }

      headSkills.lookAtBall();
      motionSkills.stand();
      CR_EXIT_SUCCESS();
    }

  private:
    LOADS_PARAMS(GoalieSaveTask, 
    {,
      (bool)(true) otherSavesEnabled,
      // (bool)(true) jumpsEnabled,
      (bool)(true) normalJumpsEnabled,
      (bool)(true) penaltyJumpsEnabled,
      (Rangef)(0.f, 80.f) standRange,
      // (Rangef)(80.f, 150.f) genuflectFromSittingRange,
      (Rangef)(80.f, 230.f) genuflectRange,
      (Rangef)(150.f, 650.f) jumpRange,
      (unsigned)(2000) waitForGenuflectMs,
      (unsigned)(3000) waitForJumpMs,
      (unsigned)(300) ballSeenSaveTimeoutMs, ///< we can only save the ball if we've seen it very recently
      (Rangef)(0.1f, 3.f) saveTimeRangeSecs, ///< we don't try to save if the ball will pass too quick or not save yet if too far in the future
    });

    READS(FieldBall);
    READS(FieldInterceptBall);
    // READS(REGameInfo);
    READS(GameState);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    bool actionDone;
    bool stayAsIs;
    // KeyframeMotionRequest::KeyframeMotionID goalieSaveMotion;
    MotionRequest::Dive::Request goalieDiveMotion;
    bool mirror;
    bool canSeeBallAfterSave;
    unsigned waitForSaveMs;


    void chooseGoalieSaveMotion(unsigned enabledSaves)
    {
      // defaults
      stayAsIs = false;
      canSeeBallAfterSave = true;
      mirror = (theFieldInterceptBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // false left, true right

      float interceptDistance =
          std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y());

      // if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectFromSitting)) &&
      //     (params.genuflectFromSittingRange.isInside(interceptDistance)))
      // {
      //   goalieSaveMotion = KeyframeMotionRequest::genuflectFromSitting; // sumo wide stance
      //   waitForSaveMs = params.waitForGenuflectMs;
      // }
      // else
      if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectStand)) &&
               (params.genuflectRange.isInside(interceptDistance)))
      {
        // goalieSaveMotion = KeyframeMotionRequest::genuflectStand; // sumo wide stance
        goalieDiveMotion = MotionRequest::Dive::squatArmsBackLeft; // sumo wide stance
        waitForSaveMs = params.waitForGenuflectMs;
      }
      else if ((params.otherSavesEnabled) && (enabledSaves & bit(Interception::genuflectStandDefender)) &&
               (params.genuflectRange.isInside(interceptDistance)))
      {
        // goalieSaveMotion = KeyframeMotionRequest::genuflectStandDefender; // sumo wide stance
        goalieDiveMotion = MotionRequest::Dive::squatWideArmsBackLeft; // sumo wide stance
        waitForSaveMs = params.waitForGenuflectMs;
      }
      // else if ((params.jumpsEnabled) && (enabledSaves & (bit(Interception::jumpLeft) | bit(Interception::jumpRight))) &&
      //          (params.jumpRange.isInside(interceptDistance)))
      // {
      //   goalieSaveMotion = KeyframeMotionRequest::keeperJumpLeft;
      //   mirror = (theFieldInterceptBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // mirror it for right jump
      //   canSeeBallAfterSave = false;
      //   waitForSaveMs = params.waitForJumpMs;
      // }
      else if ((params.normalJumpsEnabled) &&
               (enabledSaves & (bit(Interception::jumpLeft) | bit(Interception::jumpRight))) &&
               (params.jumpRange.isInside(interceptDistance)) && (theGameState.isPenaltyShootout()))
      {
        // goalieSaveMotion = KeyframeMotionRequest::keeperJumpLeft;
        // mirror = (theFieldInterceptBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // mirror it for right jump
        goalieDiveMotion = mirror ? MotionRequest::Dive::jumpRight : MotionRequest::Dive::jumpLeft;
        canSeeBallAfterSave = false;
        waitForSaveMs = params.waitForJumpMs;
      }
      // FIXME - penalty shootout - surely handled by PenaltyShotGoalie?
      // else if ((params.penaltyJumpsEnabled) && (enabledSaves & (bit(Interception::jumpLeft) | bit(Interception::jumpRight))) &&
      //          (params.jumpRange.isInside(interceptDistance)) && (theREGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT))
      // {
      //   goalieSaveMotion = KeyframeMotionRequest::keeperJumpLeft;
      //   mirror = (theFieldInterceptBall.intersectionPositionWithOwnYAxis.y() > 0) ? false : true; // mirror it for right jump
      //   canSeeBallAfterSave = false;
      //   waitForSaveMs = params.waitForJumpMs;
      // }
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
        // if ((goalieSaveMotion == KeyframeMotionRequest::genuflectFromSitting) ||
        //     (goalieSaveMotion == KeyframeMotionRequest::genuflectStand) ||
        //     (goalieSaveMotion == KeyframeMotionRequest::genuflectStandDefender))
        if ((goalieDiveMotion == MotionRequest::Dive::squatArmsBackLeft) ||
            (goalieDiveMotion == MotionRequest::Dive::squatArmsBackRight) ||
            (goalieDiveMotion == MotionRequest::Dive::squatWideArmsBackLeft) ||
            (goalieDiveMotion == MotionRequest::Dive::squatWideArmsBackRight))
        {
          float interceptDistance =
              std::fabs(theFieldInterceptBall.intersectionPositionWithOwnYAxis.y());

          bool interceptInRange = false;
          // if (goalieSaveMotion == KeyframeMotionRequest::genuflectFromSitting)
          //   interceptInRange = params.genuflectFromSittingRange.isInside(interceptDistance);
          // else
            interceptInRange = params.genuflectRange.isInside(interceptDistance);

          saveStillApplies =
              theFieldBall.ballWasSeen(params.ballSeenSaveTimeoutMs) &&
              interceptInRange &&
              params.saveTimeRangeSecs.isInside(theFieldInterceptBall.timeUntilIntersectsOwnYAxis);
        }
      }

      return  !timeDone || saveStillApplies;
    }
  };

} // namespace v2025
} // namespace RE