/**
 * @file Striker.h
 *
 * This file contains the main striker behaviours.
 * It has been adapted from the RoboEireann 2019 behaviour 
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 * @author Andy Lee Mitchell
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2024/CoroBehaviour2024.h"

#include "GotoBallSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2024
{
  /**
   * striker kickoff behaviour.
   * Pass to a teammate if at least one outfield available,
   * otherwise dribble out of the center circle
   */
  CRBEHAVIOUR(StrikerKickoffTask)
  {
    CRBEHAVIOUR_INIT(StrikerKickoffTask) {}

    void operator()(void)
    {
      bool kickSuccess = false;

      CRBEHAVIOUR_BEGIN();
        
      // only want to do this once!!
      // TODO: it would be better to choose the side based on obstacle/opponent detection
      updateRandomSide();

      if (commonSkills.isOurTeamKick()) // offense kickoff
      {
        CR_CHECKPOINT(our_kickoff);

        // FIXME: check for outfield teammates (and opponents) and decide whether to pass
        // right or left accordingly

        // if no outfield teammates, dribble out of the center circle
        // FIXME: take account of opponent positions (obstacles)


        while (true)
        {
          if (params.shotFromKickoffEnabled && kickoffShotIsViable())
          {
            gotoBallAndShootFromKickOffTask(params.minKickSectorSize);
            kickSuccess = gotoBallAndShootFromKickOffTask.isSuccess();
          }
          else
          {
            calcKickoffAngleAndType();
            gotoBallAndKickTask(targetAngle, kickType, true, kickLength);
            kickSuccess = gotoBallAndKickTask.isSuccess();
            // gotoBallAndDribbleTask(targetAngle, false, 1.f, true);
          }

          // if the ball is far enough from center point, we can assume that kick-off is finished
          // FIXME param for distance
          if (kickSuccess || theFieldBall.endPositionOnField.norm() > 350.f)
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();
        }
      }
      else // defensive kickoff
      {
        CR_CHECKPOINT(opponent_kickoff);

        // TODO: we assume we are already in a good defensive position so we don't try
        // to move any further. Is this reasonable?
        while (true)
        {
          if (theGameInfo.isBallFree()) // can we enter the centre circle and take a shot?
          {
            if (params.shotFromKickoffEnabled && kickoffShotIsViable  ())
            {
              gotoBallAndShootFromKickOffTask(params.minKickSectorSize);
              kickSuccess = gotoBallAndShootFromKickOffTask.isSuccess();
            }
            else
            {
              calcKickoffAngleAndType();
              gotoBallAndKickTask(targetAngle, kickType, true, kickLength);
              kickSuccess = gotoBallAndKickTask.isSuccess();
            }
          }
          else
          {
            headSkills.lookActive(true);
            commonSkills.stand();
          }

          if (kickSuccess || (theFieldBall.endPositionOnField.norm() > theFieldDimensions.centerCircleRadius/2.f)) // ball moved a large amount in the centre circle?
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();
        }
      }
    }

  private:
    LOADS_PARAMS(StrikerKickoffTask,
    {,
      (Vector2f) kickoffPassPosition,
      (bool) shotFromKickoffEnabled,
      (Angle) minKickSectorSize, // higher values are less likely to shoot from kickoff (but would have a better chance of succeeding)
    });

    READS(RobotPose);
    READS(GameInfo);
    READS(FieldDimensions);
    READS(FieldBall);
    READS(ExtendedGameInfo);


    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    ObstacleSkills obstacleSkills {env};

    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
    GotoBallAndShootFromKickOffTask gotoBallAndShootFromKickOffTask {env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};
    
    // bool targetLeft;
    Angle targetAngle;
    float side = 1.f;
    KickInfo::KickType kickType;
    float kickLength;

    void updateRandomSide()
    {
      if (Random::bernoulli())
        side = -1.f;
      else
        side = 1.f;
    }

    void calcKickoffAngleAndType()
    {
      Vector2f target = Vector2f(params.kickoffPassPosition.x(), side * params.kickoffPassPosition.y());

      targetAngle = theRobotPose.toRobotCoordinates(Pose2f(target.angle())).rotation;

      kickLength = target.norm();

      kickType = side < 0.f ? KickInfo::KickType::forwardFastLeft : KickInfo::KickType::forwardFastRight;
      // ACTGRAPH_FMT("target = {}", target);
      // ACTGRAPH_FMT("kickLength = {}", kickLength);
      // ACTGRAPH_FMT("targetAngle = {:.1f}_deg", targetAngle.toDegrees());
    }

    bool kickoffShotIsViable()
    {
      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
      Rangea bestGoalSector = obstacleSkills.getBestGoalSector(kickSectors);

      if (bestGoalSector.getSize() > params.minKickSectorSize)
        return true;

      return false;
    }
  };

  /**
   * striker search for ball behaviour.
   */
  CRBEHAVIOUR(StrikerSearchForBallTask)
  {
    CRBEHAVIOUR_INIT(StrikerSearchForBallTask) {}

    void operator()(bool shouldBackUp = false)
    {
      CRBEHAVIOUR_BEGIN();

      if (shouldBackUp)
      {
        // try backing up a few steps in case we lost the ball in a duel
        CR_CHECKPOINT(backUp);
        while (getCheckpointDuration() < paramBackupDurationMs)
        {
          headSkills.lookActive();
          motionSkills.walkAtRelativeSpeed(Pose2f(0, -0.8f, 0)); // walk backwards at close to full speed
          if (theFieldBall.ballWasSeen())
            CR_EXIT_SUCCESS();
          else
            CR_YIELD();
        }
      }

      // no sign of the ball, turn on the spot
      // TODO - should choose another spot if nothing found after a rotation or two

      chooseTurnDirection();

      CR_CHECKPOINT(turnOnSpotSearch);
      while (true)
      {
        lookLeftAndRightTask(turnDirection);
        motionSkills.walkAtRelativeSpeed(Pose2f(turnDirection, 0.f, 0)); // turn on the spot
        if (theFieldBall.ballWasSeen())
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }        
    }

  private:
    unsigned paramBackupDurationMs = 4000;

    READS(FieldBall);
    READS(RobotPose);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};

    LookLeftAndRightTask lookLeftAndRightTask {env};
    int turnDirection; // 1 or -1

    void chooseTurnDirection()
    {
      // Always turn in-field (towards the centre of the field) first
      // - theRobotPose is the robot relative to the field origin 
      //   and theRobotPose.inversePose is the field origin relative to the robot
      // - get the bearing to the field origin point (theRobotPose.inversePose.translation) 
      //   from the robot. (It will lie between -180 and 180 degrees)
      // - if it is between 0 and -180 the field origin is clockwise from the robot
      //   otherwise it is anticlockwise
      Angle originBearing = theRobotPose.inversePose.translation.angle();

      turnDirection = (originBearing > 0) ? 1 : -1;      
    }   
  };



  /**
   * Main entry point of the striker behaviour in playing state
   */
  CRBEHAVIOUR(StrikerPlayingTask)
  {
    CRBEHAVIOUR_INIT(StrikerPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      if (theGameInfo.isKickoff())
      {
        CR_CHECKPOINT(kickoff);
        annotation("StrikerPlayingTask kickoff");

        do
        {
          // commonSkills.activityStatus(BehaviorStatus::reKickoff);
          strikerKickoffTask();
          CR_YIELD();
        }
        while (!strikerKickoffTask.isEnded());
      }

      // at this stage we are finished with kickoff and just proceed to play.
      // The strategy is simple/naive. Go to the ball and kick it towards goal.
      // We don't even pay much attention to obstacles (unless really close) 
      // when choosing to kick at goal

      annotation("StrikerPlayingTask play normal");
      while (true)
      {
        // if ball was seen recently, we'll trust the ball model and go to
        // kick the ball at its model position towards goal

        CR_CHECKPOINT(gotoBallAndKick);
        while (theFieldBall.ballWasSeen(params.ballSeenTimeoutMs))
        {
          gotoBallAndKickBestOptionTask();
          CR_YIELD();
        }

        // if we get here, we haven't seen the ball in a while and no longer
        // believe the ball model - search for the ball

        CR_CHECKPOINT(searchForBall);
        do
        {
          strikerSearchForBallTask(theFieldBall.ballWasSeen(params.ballLostRecentlyTimeoutMs));        
          CR_YIELD();
        }
        while (!strikerSearchForBallTask.isEnded());

      }
    }

  private:
    DEFINES_PARAMS(StrikerPlayingTask, 
    {, 
      (unsigned)(5000) ballSeenTimeoutMs, // if the ball is not seen for this time we consider it lost
      (unsigned)(6000) ballLostRecentlyTimeoutMs, // if the ball *was* seen within this period we consider the ball to be recently lost
      (int)(11000) kickoffTimeMs, // after this time the ball is free and anyone can take it
    });

    READS(FieldBall);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    
    CommonSkills commonSkills {env};

    StrikerKickoffTask strikerKickoffTask {env};
    StrikerSearchForBallTask strikerSearchForBallTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
  };

} // RE2024
} // CoroBehaviour