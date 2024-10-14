/**
 * @file PenaltyShotStriker.h
 *
 * This file contains the striker behaviours to use during an in-game
 * penalty shot or a penalty shoot out after normal time.
 * 
 * It has been adapted from the RoboEireann 2019 behaviour 
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 */


#pragma once


#include "Tools/TextLogging.h"

#include "Modules/BehaviorControl/CoroBehaviour/2024/CoroBehaviour2024.h"

#include "GotoBallSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2024
{
  /**
   * During this ready state, the striker should go to penalty kick position
   * which is just outside the penalty area facing the opponent goal
   */
  CRBEHAVIOUR(PenaltyShotStrikerReadyTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotStrikerReadyTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      CR_CHECKPOINT(walk_to_penalty_ready_pose);
      while (!walkToPoseAutoAvoidanceTask.isSuccess())
      {
        // commonSkills.activityStatus(BehaviorStatus::reWalkToTacticPose);

        headSkills.lookActive();
        walkToPoseAutoAvoidanceTask(getPenaltyKickReadyPose(),
                                  Pose2f(params.speed, params.speed, params.speed),
                                  /* keepTargetRotation */ false, params.positionThreshold, params.angleThreshold);
        CR_YIELD();
      }

      CR_CHECKPOINT(stand_at_penalty_ready_pose);
      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::reStandAtTacticPose);

        headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
            Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
            theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
        commonSkills.stand();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(PenaltyShotStrikerReadyTask,
    {,
      (float)(50.f) positionThreshold, 
      (float)(5_deg) angleThreshold,
      (float)(1.f) speed, 
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(RobotDimensions);
    READS(BallSpecification);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    // bool targetLeft;
    // Angle targetAngle;

    Pose2f getPenaltyKickReadyPose()
    {
      Pose2f poseOnField = Pose2f(0_deg, theFieldDimensions.xPosOpponentPenaltyArea - theRobotDimensions.soleToFrontEdgeLength, 0);

      return theRobotPose.toRobotCoordinates(poseOnField);
    }
  };




  /**
   * This behaviour is used whether in a penalty shoot out or taking a penalty
   * shot during a game. For penalty shots we look at the ball/penalty mark position
   */
  CRBEHAVIOUR(PenaltyShotStrikerSetStateTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotStrikerSetStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
            Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
            theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
        commonSkills.stand();
        CR_YIELD();
      }
    }

  private:
    READS(FieldDimensions);
    READS(RobotPose);
    READS(BallSpecification);

    HeadSkills headSkills {env};
    CommonSkills commonSkills {env};
  };



  /**
   * This behaviour is used whether in a penalty shoot out or taking a penalty
   * shot during a game. We assume the same basic starting location in both
   * cases
   */
  CRBEHAVIOUR(PenaltyShotStrikerPlayingTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotStrikerPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // look at penalty spot for some max time (or until we see the ball - whichever is first)
      CR_CHECKPOINT(penaltyshot_look_at_pen_spot);
      while (!theFieldBall.ballWasSeen(params.ballSeenMs) && (getCheckpointDuration() < params.lookAtPenaltySpotMs))
      {
        headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
            Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
            theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
        commonSkills.stand();
        CR_YIELD();
      }

      // walk forward to the stop and scan distance

      CR_CHECKPOINT(penaltyshot_walk_near_ball);
      while (!walkToPoseNoAvoidanceTask.isSuccess())
      {
        headSkills.lookAtPoint(theRobotPose.toRobotVector3f(
            Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0),
            theBallSpecification.radius)); // look at the penalty spot (the ball should be there)
        walkToPoseNoAvoidanceTask(getStopAndScanTarget(),
                                  Pose2f(params.normalSpeed, params.normalSpeed, params.normalSpeed),
                                  /* keepTargetRotation */ true, params.positionThreshold, params.angleThreshold);
        CR_YIELD();
      }

      // do a scan (fix our location and potentially detect the goalie)

      CR_CHECKPOINT(penaltyshot_pause_and_scan);
      while (getCheckpointDuration() < params.scanDurationMs)
      {
        lookLeftAndRightTask();
        commonSkills.stand();
        CR_YIELD();
      }

      // OK, walk up and take the shot

      chooseShot();

      CR_CHECKPOINT(penaltyshot_kick);
      while (!gotoBallAndKickTask.isSuccess())
      {
        // head is managed by gotoBallAndKick
        gotoBallAndKickTask(theRobotPose.toRobotCoordinates(shotTargetOnField).angle(), shotKickType,
                            /* alignPrecisely */ true, /* length */ 6000.f, /* preStepAllowed */ true,
                            /* turnKickAllowed */ false,
                            Pose2f(params.finalLineUpSpeed, params.finalLineUpSpeed, params.finalLineUpSpeed),
                            /* ignoreObstacles */ true,
                            /* directionPrecision */ Rangea(-1_deg, 1_deg));
        CR_YIELD();
      }

      // wait until higher level behaviour switches out of this coroutine

      CR_CHECKPOINT(wait_after_kick);
      while (true)
      {
        commonSkills.standLookActive();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(PenaltyShotStrikerPlayingTask,
    {,
      (float)(250.f) stopAndScanDistance, 
      (unsigned)(1000) lookAtPenaltySpotMs,
      (unsigned)(300) ballSeenMs, ///< must see the ball within this time
      (unsigned)(2000) scanDurationMs, 
      (float)(50.f) positionThreshold, 
      (float)(5_deg) angleThreshold,
      (float)(1.f) normalSpeed, // normal speed during walk up
      (float)(0.7f) finalLineUpSpeed, // speed for close up line up
      /// the y distance inside the goal post to aim for - a tradeoff 
      /// between ease of save and the possibility of missing
      (float)(450.f) yInsideGoalPostNarrow, // easier to save 
      (float)(350.f) yInsideGoalPostWide, // might miss when aim is off
      (float)(0.7f) probabilityWideKick, // probability of aiming close to the goal post (and risking a miss)
      (float)(0.8f) probabilityAlternateSides, // probability of alternating sides rather than choosing side at random
      (bool)(true) shootLeftWithLeftFoot,
    });

    READS(FieldBall);
    READS(FieldDimensions);
    READS(RobotPose);
    READS(BallSpecification);

    CommonSkills commonSkills{env};
    HeadSkills headSkills{env};
    WalkToPoseNoAvoidanceTask walkToPoseNoAvoidanceTask{env};
    LookLeftAndRightTask lookLeftAndRightTask{env};
    GotoBallAndKickTask gotoBallAndKickTask{env};

    Vector2f shotTargetOnField;
    KickInfo::KickType shotKickType;
    int side = 0; // no prev side yet

    DECL_TLOGGER_FN(tlogger, "PenaltyShotStrikerPlaying", TextLogging::INFO)

    Pose2f getStopAndScanTarget()
    {
      return theRobotPose.toRobotCoordinates(
          Pose2f(0_deg, theFieldDimensions.xPosOpponentPenaltyMark - params.stopAndScanDistance, 0));
    }

    void chooseShot()
    {
      // we never kick straight down the middle, so choose a side
      // do we just want to go to the opposite side to the side we kicked previously?
      bool oppositeSide = (side != 0) && Random::bernoulli(params.probabilityAlternateSides);
      side = oppositeSide ? -side : (Random::bernoulli() ? 1 : -1);

      //Decide whether to shoot in more centrally or nearer the goalpost
      bool wide = Random::bernoulli((params.probabilityWideKick));
      float distInsideGoal = wide ? params.yInsideGoalPostWide : params.yInsideGoalPostNarrow;
      shotTargetOnField =
          Vector2f(theFieldDimensions.xPosOpponentGroundLine,
                   (theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius - distInsideGoal) * side);

      // if shooting left, kick with right, else kick with left. (Ideally this makes the
      // ball less likely to go wide of the outside of the foot). Note side==1 is shooting left.
      if (side == 1) // shooting to left?
        shotKickType = params.shootLeftWithLeftFoot ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      else // shooting right
        shotKickType = params.shootLeftWithLeftFoot ? KickInfo::forwardFastRightLong : KickInfo::forwardFastLeftLong;

      //TLOGI(tlogger(), "Center = {}, Side = {}, ShotKickType = {}, longShot = {}",center,side,shotKickType,longShot);
    }
  };

} // RE2024
} // CoroBehaviour2024