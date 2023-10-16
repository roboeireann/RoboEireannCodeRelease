/**
 * @file SetPlay.h
 *
 * This file implements the main set plays, i.e. kick-in, free kick, goal
 * kick etc.
 * 
 * Parts of it have been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2023
{
  /**
   * implement free kick, kick-in, possibly corner kick
   * The general approach is find the ball, go to the ball, kick to a teammate
   * since you cannot score directly
   * 
   * Not sure if goal kick should be here also since you would imagine that
   * should be the goalie rather than the striker taking it
   * 
   * TODO: actually a robot other than the goalie can take a goal kick
   */
  CRBEHAVIOUR(SetPlayStrikerFreeKickTask)
  {
    CRBEHAVIOUR_INIT(SetPlayStrikerFreeKickTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(goto_ball_and_kick_indirect);
        while (theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
        {
          // commonSkills.activityStatus(BehaviorStatus::reIndirectKick);
          gotoBallAndKickBestOptionTask(/* indirect */ true);
          CR_YIELD();
        }

        CR_CHECKPOINT(searchForBall);
        while (!strikerSearchForBallTask.isEnded())
        {
          // commonSkills.activityStatus(BehaviorStatus::reSearchForBall);
          strikerSearchForBallTask(theFieldBall.ballWasSeen(params.ballLostRecentlyTimeoutMs));        
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(SetPlayStrikerFreeKickTask, 
    {, 
      (unsigned)(5000) ballSeenTimeoutMs, // if the ball is not seen for this time we consider it lost
      (unsigned)(6000) ballLostRecentlyTimeoutMs, // if the ball *was* seen within this period we consider the ball to be recently lost
    });

    READS(FieldBall);
    CommonSkills commonSkills {env};

    StrikerSearchForBallTask strikerSearchForBallTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
  };


  // ==========================================================================

  CRBEHAVIOUR(SetPlayStrikerCornerSearchTask)
  {
    CRBEHAVIOUR_INIT(SetPlayStrikerCornerSearchTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      chooseCorner(true);

      CR_CHECKPOINT(GotoFirstCorner);
      while (!walkToPoseAutoAvoidanceTask.isEnded())
      {
        commonSkills.lookActive();
        walkToPoseAutoAvoidanceTask(cornerPose, Pose2f(params.speed, params.speed, params.speed));
        CR_YIELD();
      }

      chooseCorner(false);

      CR_CHECKPOINT(GotoSecondCorner);
      while (true)
      {
        commonSkills.lookActive();
        walkToPoseAutoAvoidanceTask(cornerPose, Pose2f(params.speed, params.speed, params.speed));
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(SetPlayStrikerCornerSearchTask, 
    {, 
      (unsigned)(2000) waitMs, // if the ball is not seen for this time we consider it lost
      (float)(1.0f) speed,
    });

    READS(FieldBall);
    READS(FieldDimensions);
    READS(RobotPose);

    CommonSkills commonSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};


    Pose2f cornerPose;
    Vector2f corner;

    void chooseCorner(bool closestFirst = false)
    {
      Vector2f cornerLeft = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftSideline);
      Vector2f cornerRight = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightSideline);

      if (closestFirst)
      {
        if ((cornerLeft - theRobotPose.translation).squaredNorm() < (cornerRight - theRobotPose.translation).squaredNorm())
          corner = cornerLeft;
        else
          corner = cornerRight;
      }
      else if (corner == cornerLeft)
        corner = cornerRight;
      else
        corner = cornerLeft;

      // we have picked a corner, so choose pose to look at it

      if (corner.y() > 0) // left?
        cornerPose = Pose2f(45_deg, corner.x() - 2000.f, corner.y() - 2000.f);
      else
        cornerPose = Pose2f(-45_deg, corner.x() - 2000.f, corner.y() + 2000.f);
    }
  };

  // ==========================================================================

  CRBEHAVIOUR(SetPlayStrikerCornerKickTask)
  {
    CRBEHAVIOUR_INIT(SetPlayStrikerCornerKickTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(goto_ball_and_kick_indirect);
        while (theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
        {
          // commonSkills.activityStatus(BehaviorStatus::reIndirectKick);
          gotoBallAndKickBestOptionTask(/* indirect */ true);
          CR_YIELD();
        }

        // if we get here, the ball was not seen

        CR_CHECKPOINT(search_for_corner_ball);
        while (!theFieldBall.ballWasSeen())
        {
          // commonSkills.activityStatus(BehaviorStatus::reSearchForBall);
          setPlayStrikerCornerSearchTask();
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(SetPlayStrikerCornerKickTask, 
    {, 
      (unsigned)(5000) ballSeenTimeoutMs, // if the ball is not seen for this time we consider it lost
      (unsigned)(6000) ballLostRecentlyTimeoutMs, // if the ball *was* seen within this period we consider the ball to be recently lost
    });

    READS(FieldBall);
    CommonSkills commonSkills {env};

    SetPlayStrikerCornerSearchTask setPlayStrikerCornerSearchTask {env};
    StrikerSearchForBallTask strikerSearchForBallTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
  };


  // ==========================================================================



  /**
   * implement free kick, kick-in, possibly corner kick for supporters
   * The general approach is go to a decent position (at least one robot in
   * a position to receive the pass)
   */
  CRBEHAVIOUR(SetPlaySupporterPlayingTask)
  {
    CRBEHAVIOUR_INIT(SetPlaySupporterPlayingTask) {}

    void operator()(void)
    {
      // commonSkills.activityStatus(BehaviorStatus::reSetPlayBackoffAndStand);

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(gettingToKickInTacticPose);
        supporterKickInTacticsTask();
        CR_YIELD();
      }
    }

  private:
    SupporterKickInTacticsTask supporterKickInTacticsTask {env};
  };


} // RE2023
} // CoroBehaviour
