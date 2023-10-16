/**
 * @file PlayingState.h
 *
 * This task implements the top level playing state behaviour for all 
 * attacker robots in the dynamic ball handling challenge.
 *
 * @author Rudi Villing
 * @author Danny Ryan
 * @author Andy Lee Mitchell
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/InterceptSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/BallAndKickSkills.h"

#include "Tools/TextLogging.h"


namespace CoroBehaviour
{
namespace RC2023
{
  /**
   * Supporter behaviour
   */

  CRBEHAVIOUR(AttackingSupporterPlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingSupporterPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      if (theExtendedGameInfo.timeSincePlayingStarted < static_cast<int>(params.walkToStartingPoseMs))
      {
        CR_CHECKPOINT(walk_to_starting_pose);
        setStartingPose(); // set it once and don't adjust it any more
        while ((getCheckpointDuration() < params.walkToStartingPoseMs) && !walkToPoseAutoAvoidanceTask.isSuccess())
        {
          headSkills.lookActive();
          walkToPoseAutoAvoidanceTask(theRobotPose.toRobotCoordinates(supporterPose),
                                      Pose2f(params.fast, params.fast, params.fast),
                                      /* keepTargetRotation */ true, params.positionThreshold, params.angleThreshold);
          CR_YIELD();
        }

        CR_CHECKPOINT(wait_for_first_pass);
        while ((theTeamBehaviorStatus.passStatus.passReceiver == -1) ||
               (theTeamBehaviorStatus.passStatus.passReceiver == theRobotInfo.number) ||
               (theFieldBall.timeSinceTeamBallWasValid > 1000))
        {
          if (interceptSkills.isInterceptNeeded())
            interceptTask(InterceptTask::WALK_INTERCEPTS);
          else
          {
            headSkills.lookActive();          
            commonSkills.stand();
          }
          CR_YIELD();
        }
      }


      CR_CHECKPOINT(goto_next_pass_position);
      setNextReceivePose(); // set it once and don't adjust it any more
      walkToPoseAutoAvoidanceTask.reset(); // ensure that we don't just consider walk to first pose
      
      while (!walkToPoseAutoAvoidanceTask.isSuccess())
      {
        if (interceptSkills.isInterceptNeeded())
          interceptTask(InterceptTask::WALK_INTERCEPTS);
        else
        {
          headSkills.lookActive();
          walkToPoseAutoAvoidanceTask(theRobotPose.toRobotCoordinates(supporterPose),
                                    Pose2f(params.fast, params.fast, params.fast),
                                    /* keepTargetRotation */ true, params.positionThreshold, params.angleThreshold);
        }
        CR_YIELD();
      }
      
      // wait until higher level behaviour switches out of this coroutine,
      // e.g. by changing the robot role to ball player

      CR_CHECKPOINT(wait_for_ball);
      // While ball not seen, look straight ahead
      while(true)
      {
        if (interceptSkills.isInterceptNeeded())
          interceptTask(InterceptTask::WALK_INTERCEPTS);
        else
        {
          headSkills.lookActive();          
          commonSkills.stand();
        }
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(AttackingSupporterPlayingTask,
    {,
      (unsigned)(5000) lookForDefenderMs,
      (unsigned)(7000) walkToStartingPoseMs,
      (float)(50.f) positionThreshold, 
      (float)(5_deg) angleThreshold,
      (float)(1.f) fast, 
    });
    
    READS(FrameInfo);
    READS(TeamBehaviorStatus);
    READS(RobotPose);
    READS(FieldDimensions);
    READS(FieldBall);
    READS(RobotInfo);
    READS(ExtendedGameInfo);
    
    CommonSkills commonSkills {env};
    MotionSkills motionSkills {env};
    HeadSkills headSkills {env};
    ObstacleSkills obstacleSkills {env};
    InterceptSkills interceptSkills {env};
    InterceptTask interceptTask {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    WalkToPoseNoAvoidanceTask walkToPoseNoAvoidanceTask {env};
    TurnToPointOdometryTask turnToPointOdometryTask {env};

    Pose2f supporterPose;

    void setStartingPose()
    {
      float side = (theRobotPose.translation.y() > 0) ? 1.f : -1.f;

      float yOffset = (0.5f * (theFieldDimensions.yPosLeftSideline + theFieldDimensions.centerCircleRadius)) + 100.f; // extra room for ball inaccuracy

      supporterPose = Pose2f(-110_deg * side, 250.f, yOffset * side);
    }

    void setNextReceivePose()
    {
      float side = (theRobotPose.translation.y() > 0) ? 1.f : -1.f;
      float xOffset = 0.3f * theFieldDimensions.xPosOpponentPenaltyMark;

      supporterPose = Pose2f(-90_deg * side, xOffset, 1.6f * theFieldDimensions.yPosLeftGoalArea * side);
    }
  };

  /**
   * Ball player behaviour
   */

  CRBEHAVIOUR(AttackingBallPlayerPlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingBallPlayerPlayingTask) {}

    void operator()(int prevPassKicker, int prevPassReceiver)
    {
      CRBEHAVIOUR_LOOP()
      {
        addActivationGraphOutput(fmt::format("prevPassKicker {}, receiver {}", prevPassKicker, prevPassReceiver));

        if (theRobotInfo.number == 1)
        {
          CR_CHECKPOINT(pass_to_winger);

          // choose initial pass target and don't change once decided
          chooseFirstPassTarget();

          while (!gotoBallAndPassTask.isSuccess())
          {
            commonSkills.passTargetStatus(passTargetPlayer, passTargetOnField);
            gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
            CR_YIELD();
          }
        }


        (void)prevPassReceiver; // force variable use
        if (prevPassKicker == 1) // if it kicked from robot1, we're clear to make the second pass
        {
          CR_CHECKPOINT(make_second_pass);

          while (!gotoBallAndPassTask.isSuccess())              
          {
            chooseAdaptivePassTarget();
            commonSkills.passTargetStatus(passTargetPlayer, passTargetOnField);
            gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
            CR_YIELD();
          }
        }

        CR_CHECKPOINT(optional_third_pass);

        checkOpponentPos();
        while(opponentInWay)
        {
            checkOpponentPos();
            if (!opponentInWay)
            {
              headSkills.lookActive();
              commonSkills.stand();
              CR_YIELD();
            }

            chooseAdaptivePassTarget();
            commonSkills.passTargetStatus(passTargetPlayer, passTargetOnField);
            gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
            CR_YIELD();
        }

        // if we get this far, we're clear to shoot

        (void)prevPassReceiver; // force variable use
        if (prevPassKicker == (theRobotInfo.number == 2 ? 3 : 2)) // if it kicked from the other attacking robot we are clear to shoot
        {
          CR_CHECKPOINT(shoot_at_goal);

          while (!theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField) && !opponentInWay)
          {
            checkOpponentPos();
            gotoBallAndShootTask();
            CR_YIELD();
          }
        }
        
        // wait until higher level behaviour switches out of this coroutine
        CR_CHECKPOINT(wait_after_kick);
        while ((getCheckpointDuration() < 1000) || (theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField)))
        {
          headSkills.lookActive();
          commonSkills.stand();
          CR_YIELD();
        }
      }

    }

  private:
    DEFINES_PARAMS(AttackingBallPlayerPlayingTask, 
    {, 
      (float)(1.f) speed1, 
    });

    READS(RobotPose);
    READS(FieldBall);
    READS(FieldDimensions);
    READS(RobotInfo);
    READS(TeamData);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
    ObstacleSkills obstacleSkills{env};
    GotoBallAndPassTask gotoBallAndPassTask{env};
    GotoBallAndShootTask gotoBallAndShootTask{env};

    int passTargetPlayer; // the player number we want to pass to
    Vector2f passTargetOnField;
    bool opponentInWay;


    void chooseFirstPassTarget()
    {
      float yOffset = (0.5f * (theFieldDimensions.yPosLeftSideline + theFieldDimensions.centerCircleRadius));
      
      if (Random::bernoulli())
      {
        passTargetPlayer = 2;
        passTargetOnField = Vector2f(250.f, yOffset - 150.f); // offset to place the ball in front of the player
      }
      else
      {
        passTargetPlayer = 3;
        passTargetOnField = Vector2f(250.f, -(yOffset - 150.f));
      }
    }

    void chooseSecondPassTarget()
    {
      float side = (theRobotPose.translation.y() > 0) ? 1.f : -1.f;

      float xOffset1 = 0.6f * theFieldDimensions.xPosOpponentPenaltyMark;
      float yOffset1 = 1.6f * theFieldDimensions.yPosLeftGoalArea;

      passTargetPlayer = (theRobotInfo.number == 2) ? 3 : 2;
      passTargetOnField = Vector2f(xOffset1, (yOffset1 -250.f) * -side);
    }

    void chooseAdaptivePassTarget()
    {
      passTargetPlayer = (theRobotInfo.number == 2) ? 3 : 2;

      // get position of pass target
      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (teammate.number == passTargetPlayer)
        {
          passTargetOnField = teammate.theRobotPose.translation;
          passTargetOnField.x() += 300.f; // pass ahead of the player
          return;
        }
      }

      chooseSecondPassTarget(); // fall back to hardcoded version
    }

    void checkOpponentPos()
    {
        opponentInWay = obstacleSkills.isOpponentDangerNearBall(1500.f);
        
        // HACK - shoot if we're far enough up the pitch that making another pass wouldn't be worth it
        if (theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f))
          opponentInWay = false;
    
        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectors(theRobotPose.translation);
        Rangea testRange = Rangea(-70_deg, 70_deg);
        std::tuple<Rangea, bool> bestSector = obstacleSkills.getBestFreeSector(kickSectors, testRange);

        if (std::get<1>(bestSector))
          opponentInWay = false;
    }
  };

  /**
   * Goalie default behaviour
   */
  CRBEHAVIOUR(AttackingGoaliePlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingGoaliePlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        headSkills.lookActive();
        commonSkills.stand();
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
  };

  /**
   * This is the entry point task for the playing state. It decides which
   * specialized behaviour to run
   */
  CRBEHAVIOUR(AttackerPlayingStateTask)
  {
    CRBEHAVIOUR_INIT(AttackerPlayingStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theTeamBehaviorStatus.role.playsTheBall())
          attackingBallPlayerPlayingTask(prevPassKicker, prevPassReceiver);
        else if (theTeamBehaviorStatus.role.isGoalkeeper())
          attackingGoaliePlayingTask();
        else
          attackingSupporterPlayingTask();

        if ((theTeamBehaviorStatus.passStatus.passReceiver != prevPassReceiver) && (theTeamBehaviorStatus.passStatus.passReceiver != -1))
        {
          prevPassKicker = theTeamBehaviorStatus.passStatus.passKicker;
          prevPassReceiver = theTeamBehaviorStatus.passStatus.passReceiver;
        }

        CR_YIELD();
      }
    }

  private:
    READS(TeamBehaviorStatus);

    AttackingBallPlayerPlayingTask attackingBallPlayerPlayingTask {env};
    AttackingGoaliePlayingTask attackingGoaliePlayingTask {env};
    AttackingSupporterPlayingTask attackingSupporterPlayingTask {env};

    int prevPassKicker = -1;
    int prevPassReceiver = -1;
  };
} // RC2023
} // CoroBehaviour2023
