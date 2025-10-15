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

#include "Modules/BehaviorControl/CoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

#include "GotoBallSkills.h"
#include "Striker.h"
#include "Supporter.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2025
{
  /**
   * Implement the set play behaviour to look at the referee for the visual signal
   */
  CRBEHAVIOUR(SetPlayLookAtRefereeTask)
  {
    CRBEHAVIOUR_INIT(SetPlayLookAtRefereeTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // goalkeeper should not do this
        CR_CHECKPOINT(goalkeeper_set_play);
        if (theTeamBehaviorStatus.role.isGoalkeeper())
        {
          // commonSkills.activityStatus(BehaviorStatus::reGoalkeeperSetPlay);
          gotoTacticPoseTask();
          CR_YIELD();
        }
        // if the robot is in a position where it can see the referee
        CR_CHECKPOINT(look_at_referee);
        while(canSeeReferee())
        {
          headSkills.lookAtPoint(getBestViewPosition(), /* speed */ 180_deg, HeadMotionRequest::upperCamera);
          motionSkills.stand(/* high: */ true);
          CR_YIELD();
        }
        // turn to referee
        CR_CHECKPOINT(turn_to_referee);
        while (!canSeeReferee() && theExtendedGameInfo.timeSinceFreeKickStarted < params.goToTacticPoseTimeoutMs)
        {
          // commonSkills.activityStatus(BehaviorStatus::reLookAtReferee);
          headSkills.lookActive();
          walkToPoseAutoAvoidanceTask(getTargetPose(), Pose2f(1.f, 1.f, 1.f));
          CR_YIELD();
        }
        // robot cant see the referee, get into tactic pose
        CR_CHECKPOINT(goto_tactic_pose);
        gotoTacticPoseTask();
        CR_YIELD();
      }
    }
  private:
    DEFINES_PARAMS(SetPlayLookAtRefereeTask, 
    {, 
      (unsigned)(7000) timeSinceFreeKickStarted, // if the time since the free kick started is greater than this we consider the referee signal to be valid
      (int)(3000) goToTacticPoseTimeoutMs, // if the robot cannot see the referee for this time we consider it to be lost
    });
    READS(RobotPose);
    READS(FieldDimensions);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    READS(TeamBehaviorStatus);
    // READS(SetPlayGestureDetector);
    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};
    GotoTacticPoseTask gotoTacticPoseTask {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Vector2f refPointOnField = Vector2f(0,theFieldDimensions.yPosLeftSideline + 300.f);
    

    bool canSeeReferee()
    {
      updateRefPosition();

      float distanceToReferee = (theRobotPose.translation - refPointOnField).norm();
      if (distanceToReferee < 1500.f)
      {
        // robot is too close to the referee, cannot see them
        return false;
      }

      // check if the robot can see the referee
      Pose2f refPoint = theRobotPose.toRobotCoordinates(Pose2f(getRefAngle(), refPointOnField));
      float angleThreshold = 50_deg; // angle threshold to consider the referee visible
      return abs(refPoint.rotation) < angleThreshold;
    }

    Pose2f getTargetPose()
    {
      updateRefPosition();
      // turn the robot to face the referee
      
      Pose2f refPoint = theRobotPose.toRobotCoordinates(Pose2f(getRefAngle(), refPointOnField));
      Pose2f targetPose = Pose2f(refPoint.rotation, theRobotPose.translation.x(), theRobotPose.translation.y());
      // targetPose.rotation = Angle::fromDegrees(0.f); // face the referee
      return targetPose;
    }

    Vector3f getBestViewPosition() 
    {
      updateRefPosition();
      // return Vector3f(0, theFieldDimensions.yPosLeftSideline + 300, 1200.f); // looking approx the center of the height
      Vector2f refPoint = theRobotPose.toRobotCoordinates(refPointOnField);
      
      return Vector3f(refPoint.x(), refPoint.y(), 1200.f); // looking approx the center of the height
    }

    float getRefAngle()
    {
      if (theGameInfo.isPlayingLeftToRight())
        return pi_2; // face the referee on the left side
      else
        return -pi_2; // face the referee on the right side
      
    }

    void updateRefPosition()
    {
      if(theGameInfo.isPlayingLeftToRight())
        refPointOnField.y() = theFieldDimensions.yPosLeftSideline + 300.f;
      else
        refPointOnField.y() = theFieldDimensions.yPosRightSideline + 300.f;
    }
  };

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
          // commonSkills.recordActivity(BehaviorStatus::reIndirectKick);
          gotoBallAndKickSetPlayTask();
          CR_YIELD();
        }

        CR_CHECKPOINT(searchForBall);
        while (!strikerSearchForBallTask.isEnded())
        {
          // commonSkills.recordActivity(BehaviorStatus::reSearchForBall);
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
    GotoBallAndKickSetPlayTask gotoBallAndKickSetPlayTask {env};
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
          // commonSkills.recordActivity(BehaviorStatus::reIndirectKick);
          gotoBallAndKickSetPlayTask();
          CR_YIELD();
        }

        // if we get here, the ball was not seen

        CR_CHECKPOINT(search_for_corner_ball);
        while (!theFieldBall.ballWasSeen())
        {
          // commonSkills.recordActivity(BehaviorStatus::reSearchForBall);
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
    GotoBallAndKickSetPlayTask gotoBallAndKickSetPlayTask {env};
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
      // commonSkills.recordActivity(BehaviorStatus::reSetPlayBackoffAndStand);

      CRBEHAVIOUR_LOOP()
      {
        // CR_CHECKPOINT(gettingToKickInTacticPose);
        // supporterKickInTacticsTask();
        gotoTacticPoseTask();
        CR_YIELD();
      }
    }

  private:
    SupporterKickInTacticsTask supporterKickInTacticsTask {env};
    GotoTacticPoseTask gotoTacticPoseTask {env};
  };


} // RE2025
} // CoroBehaviour
