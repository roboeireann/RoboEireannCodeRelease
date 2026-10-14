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

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/ObstacleSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/InterceptSkills.h"
// #include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025DynamicBallHandling/InterceptSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025DynamicBallHandling/BallAndKickSkills.h"

#include "TextLogging/TextLogging.h"


namespace RE
{
namespace v2025 
{ 
namespace DBHC
{
  CRBEHAVIOUR(GoToPoseWithRepositionTask)
  {
    CRBEHAVIOUR_INIT(GoToPoseWithRepositionTask) {}

    void operator()(Pose2f& targetPose, float walkSpeed, HystThresholdsf distanceThreshold, HystThresholdsa angleThreshold)
    {
      CRBEHAVIOUR_LOOP()
      {
        // walk to tactic (formation) pose and stand, looking active

        CR_CHECKPOINT(walk_to_pose_repositioning);
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          headSkills.lookActive(/* withBall: */ true);
          walkToPoseAutoAvoidanceTask(targetPose, Pose2f(walkSpeed, walkSpeed, walkSpeed),
                                      /* keepTargetRotation */ true, distanceThreshold.lower, angleThreshold.lower);
          CR_YIELD();
        }

        CR_CHECKPOINT(stand_at_pose);
        while (commonSkills.isPoseClose(targetPose, distanceThreshold.upper, angleThreshold.upper))
        {
          headSkills.lookActive(/* withBall: */ true);
          commonSkills.stand();
          CR_YIELD();
        }

        // no longer close to targetPose, so back to repositioning
      }
    }

  private:
    TextLogger tlogger = TextLogging::get("GoToPoseWithRepositionTask", TextLogging::VERBOSE);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
  };


  // ------------------------------------------------------------------------
  // Supporter behaviour (Just player 1 once it has kicked)
  // ------------------------------------------------------------------------

  CRBEHAVIOUR(AttackingSupporterPlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingSupporterPlayingTask) {}

    void operator()(void)
    {
      tacticPose = theRobotPose.toRobotCoordinates(theREActiveTactic.formationPose);

      CRBEHAVIOUR_LOOP()
      {
        // goToPoseWithRepositionTask(tacticPose, params.walkSpeed, params.distanceThreshold, params.angleThreshold);
        headSkills.lookActive(/* withBall: */ true);
        walkToPoseTask(tacticPose, params.walkSpeed, AVOIDANCE_AUTO, /* keepTargetRotation */ true,
                        /* standAtTarget */ true, params.distanceThresholds, params.angleThresholds);

        CR_YIELD();
      }
    }

  private:
    TextLogger tlogger = TextLogging::get("AttackingSupporterPlayingTask", TextLogging::VERBOSE);

    DEFINES_PARAMS(AttackingSupporterPlayingTask,
    {,
      (float)(1.f) walkSpeed,
      (HystThresholdsf)(100.f, 150.f) distanceThresholds, // mm
      (HystThresholdsa)(5_deg, 10_deg) angleThresholds,
    });

    READS(RobotPose);
    // READS(FieldBall);
    READS(REActiveTactic);
    
    HeadSkills headSkills {env};
    // GoToPoseWithRepositionTask goToPoseWithRepositionTask {env};
    WalkToPoseTask walkToPoseTask {env};

    Pose2f tacticPose; // robot relative
  };

  /**
  * The receiver walks to the designated formation pose (possibly modified if needed to deal with opponents).
  * Then the receiver awaits the pass and intercepts as needed.
  */

  CRBEHAVIOUR(AttackingReceiverPlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingReceiverPlayingTask) {}

    void operator()(void)
    {
      tacticPose = theRobotPose.toRobotCoordinates(theREActiveTactic.formationPose);

      CRBEHAVIOUR_LOOP()
      {
        if (interceptTask.isInterceptNeeded())
          CR_WHILE(interceptTask.isInterceptNeeded(), interceptTask(InterceptTask::WALK_INTERCEPTS));
        else
        {
          headSkills.lookActive(/* withBall: */ true);
          walkToPoseTask(tacticPose, params.walkSpeed, AVOIDANCE_AUTO, /* keepTargetRotation */ true,
                         /* standAtTarget */ true, params.distanceThresholds, params.angleThresholds);
          CR_YIELD();
        }
      }

//     // walk to tactic (formation) pose waiting to receive a pass and stand, looking active
//       // (we assume we can complete this before the ball is passed to us)
// 
//       CR_CHECKPOINT(walk_to_tactic_pose);
//       do
//       {
//         headSkills.lookActive(/* withBall: */ true);
//         walkToPoseTask(tacticPose, params.walkSpeed, AVOIDANCE_AUTO, /* keepTargetRotation */ true,
//                        params.distanceThresholds, params.angleThresholds);
// 
//         // walkToPoseAutoAvoidanceTask(tacticPose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
//         //                             /* keepTargetRotation */ true, params.distanceThreshold.lower,
//         //                             params.angleThreshold.lower);
//         CR_YIELD();
//       } while (!walkToPoseAutoAvoidanceTask.isSuccess());
// 
//           // we wait at this checkpoint until the ball is passed to us (waiting indefinitely if needed).
//           // Once the ball is passed, the higher level behaviour will switch out of this coro to something else
// 
//           CR_CHECKPOINT(wait_for_pass);
//       while (true)
//       {
//         if (interceptSkills.isInterceptNeeded())
//           interceptTask(InterceptTask::WALK_INTERCEPTS);
//         else
//           goToPoseWithRepositionTask(tacticPose, params.walkSpeed, params.distanceThreshold, params.angleThreshold);
// 
//         CR_YIELD();
//       }
    }

  private:
    DEFINES_PARAMS(AttackingReceiverPlayingTask,
    {,
      (float)(1.f) walkSpeed,
      (HystThresholdsf)(100.f, 150.f) distanceThresholds, // mm
      (HystThresholdsa)(5_deg, 10_deg) angleThresholds,
    });
    
    READS(RobotPose);
    READS(REActiveTactic);
    
    HeadSkills headSkills {env};
    // InterceptSkills interceptSkills {env};
    InterceptTask interceptTask {env};
    GoToPoseWithRepositionTask goToPoseWithRepositionTask {env};
    // WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    WalkToPoseTask walkToPoseTask {env};

    Pose2f tacticPose; // robot relative
  };

  // ------------------------------------------------------------------------
  // Ball player behaviour
  // ------------------------------------------------------------------------

  /**
  * Robot 1 should look at the position of defender 2 (the one on the edge of the opposition box).
  * It should pass to the same side of the pitch that the defender 2, in relation to it's y-axis,
  * in order to give more space for the second supporter to have more space for a successful shot. 
  */

  CRBEHAVIOUR(AttackingBallPlayerPlayingTask)
  {
    CRBEHAVIOUR_INIT(AttackingBallPlayerPlayingTask) {}

    void operator()(int prevPassKicker, int prevPassReceiver)
    {
      CRBEHAVIOUR_BEGIN();

      ACTGRAPH_FMT("prevPassKicker {}, receiver {}", prevPassKicker, prevPassReceiver);

      if (theGameState.playerNumber == 1)
      {
        // Robot 1 make the first pass to either winger
        CR_CHECKPOINT(pass_to_winger);

        // choose initial pass target and don't change once decided
        chooseFirstPassTarget();

        while (!gotoBallAndPassTask.isSuccess())
        {
          commonSkills.recordPassTarget(passTargetPlayer, passTargetOnField);
          gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          CR_YIELD();
        }
      }

      // while ball position is not close enough to goal, pass back and forth
      while (theFieldBall.endPositionOnField.x() < 1500.f)
      {
        CR_CHECKPOINT(intercept_if_needed);

        if (interceptTask.isInterceptNeeded())
          CR_WHILE(interceptTask.isInterceptNeeded(), interceptTask(InterceptTask::WALK_INTERCEPTS));

        CR_CHECKPOINT(dribble_forward);
        chooseDribbleForwardTarget();
        CR_UNTIL(theFieldBall.endPositionOnField.x() >= dribbleTargetOnField.x(),
                 gotoBallAndDribbleTask(getDribbleTargetAngle(), /* ignoreObstacles */ false, /* kickPower */ 0.5f));

                 
        CR_CHECKPOINT(pass_to_teammate);
        do
        {
          chooseAdaptivePassTarget();

          commonSkills.recordPassTarget(passTargetPlayer, passTargetOnField);
          gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          CR_YIELD();
        } while (!gotoBallAndPassTask.isSuccess());
      }

      if (checkOpponentInWay())
      {
        CR_CHECKPOINT(optional_pass_if_opponent_in_Way);
        do
        {
          chooseAdaptivePassTarget();
          commonSkills.recordPassTarget(passTargetPlayer, passTargetOnField);
          gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          CR_YIELD();
        } while (checkOpponentInWay());          
      }

        
      // if we get this far, we're clear to shoot
      CR_CHECKPOINT(shoot_at_goal);
      CR_UNTIL(theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField), gotoBallAndShootTask());

      // wait until higher level behaviour switches out of this coroutine
      CR_CHECKPOINT(wait_after_kick);
      CR_WHILE(true, commonSkills.standLookActive());
    }

  private:
    DEFINES_PARAMS(AttackingBallPlayerPlayingTask, 
    {, 
      (float)(1.f) speed1, 
    });

    READS(RobotPose);
    READS(FieldBall);
    READS(FieldDimensions);
    READS(RETeamData);
    READS(GameState);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
    ObstacleSkills obstacleSkills{env};
    GotoBallAndPassTask gotoBallAndPassTask{env};
    GotoBallAndShootTask gotoBallAndShootTask{env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};
    InterceptTask interceptTask {env};

    // GoToBallAndPassToTeammateTask goToBallAndPassToTeammateTask{env};
    // GotoBallAndKickTask gotoBallAndKickTask{env};
    

    int passTargetPlayer; // the player number we want to pass to
    Vector2f passTargetOnField;

    Vector2f shotTargetOnField;
    // KickInfo::KickType shotKickType;
    // float side;
    bool opponentInWay;

    Vector2f dribbleTargetOnField;
    Angle dribbleAngle;


    void chooseDribbleForwardTarget()
    {
      dribbleTargetOnField = theFieldBall.endPositionOnField;
      dribbleTargetOnField.x() += 700.f;
    }

    Angle getDribbleTargetAngle()
    {
      dribbleAngle = theRobotPose.toRobotCoordinates(dribbleTargetOnField).angle();
      return dribbleAngle;
    }

    void chooseFirstPassTarget()
    {
      // CAUTION: match with setStartingPose above
      //if (obstacleSkills.nearestOpponentDistance(2) < obstacleSkills.nearestOpponentDistance(3))
      float yOffset = (0.5f * (theFieldDimensions.yPosLeftTouchline + theFieldDimensions.centerCircleRadius));
      
      // passTargetPlayer = 2;
      // passTargetOnField = Vector2f(250.f, yOffset - 150.f); // offset to place the ball in front of the player

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


//     void chooseSecondPassTarget()
//     {
//       // CAUTION: match with setNextReceivePose
//       float side = (theRobotPose.translation.y() > 0) ? 1.f : -1.f;
// 
//       float xOffset1 = 0.6f * theFieldDimensions.xPosOpponentPenaltyMark;
//       float yOffset1 = 1.6f * theFieldDimensions.yPosLeftGoalArea;
//       // if the robot is close to the half way line, move to forward receive pose
// 
// 
//       passTargetPlayer = (theGameState.playerNumber == 2) ? 3 : 2;
//       passTargetOnField = Vector2f(xOffset1, (yOffset1 -250.f) * -side);
//     }
// 
    void chooseAdaptivePassTarget()
    {
      passTargetPlayer = (theGameState.playerNumber == 2) ? 3 : 2;

      // get position of pass target

      for (const RETeammate& teammate : theRETeamData.teammates)
      {
        if (teammate.number == passTargetPlayer)
        {
          passTargetOnField = teammate.theRobotPose.translation;
          passTargetOnField.x() += 300.f; // pass ahead of the player
          return;
        }
      }

      // chooseSecondPassTarget(); // fall back to hardcoded version
    }

    // void chooseShot()
    // {
    //   //float side = Random::bernoulli() ? 1 : -1;
    //   side = 1;
    //   // if (obstacleSkills.areOpponentsNearPointOnField(Vector2f(2850.0f, -1000.0f)))
    //   //   side = -1;

    //   // else
    //   //   side = 1;

    //   // if shooting left, kick with the left, otherwise kick with the right
    //   shotKickType = (side == -1) ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
    //   shotTargetOnField = Vector2f(theFieldDimensions.xPosHalfwayLine, side*0.7f*(theFieldDimensions.yPosLeftTouchline));
    // }

    bool checkOpponentInWay()
    {
      opponentInWay = obstacleSkills.isOpponentDangerNearBall(1500.f);
      // opponentInWay = obstacleSkills.areOpponentsNearPointOnField(Vector2f(theRobotPose.translation.x(),
      // theFieldDimensions.yPosRightGoalArea));

      // HACK - shoot if we're far enough up the pitch that making another pass wouldn't be worth it
      if (opponentInWay && (theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f)))
        opponentInWay = false;

      if (opponentInWay)
      {
        const std::list<SectorWheel::Sector> &kickSectors =
            obstacleSkills.populateKickSectors(theRobotPose.translation);

        // std::list<SectorWheel::Sector> goalSectors = obstacleSkills.getGoalSectors(kickSectors, 5_deg);

        // if (goalSectors.size() < 2) // FIXME this should be a param
        //   opponentInWay = true;

        Rangea testRange = Rangea(-70_deg, 70_deg);
        std::tuple<Rangea, bool> bestSector = obstacleSkills.getBestFreeSector(kickSectors, testRange);

        if (std::get<1>(bestSector))
          opponentInWay = false;
      }

      return opponentInWay;
    }
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
        if (theREActiveTactic.situationRole == PlayerRole::ballPlayer)
          attackingBallPlayerPlayingTask(prevPassKicker, prevPassReceiver);
        else if (theGameState.playerNumber == 1) // if player 1 is no longer the ball player, just support
          attackingSupporterPlayingTask();
        else
          attackingReceiverPlayingTask();

        // if ((theTeamBehaviorStatus.passStatus.passReceiver != prevPassReceiver) && (theTeamBehaviorStatus.passStatus.passReceiver != -1))
        // {
        //   prevPassKicker = theTeamBehaviorStatus.passStatus.passKicker;
        //   prevPassReceiver = theTeamBehaviorStatus.passStatus.passReceiver;
        // }

        CR_YIELD();
      }
    }

  private:
    READS(REActiveTactic);
    READS(GameState);

    AttackingBallPlayerPlayingTask attackingBallPlayerPlayingTask {env};
    AttackingReceiverPlayingTask attackingReceiverPlayingTask {env};
    AttackingSupporterPlayingTask attackingSupporterPlayingTask {env};

    int prevPassKicker = -1;
    int prevPassReceiver = -1;
  };

} // DBHC
} // namespace v2025
} // namespace RE
