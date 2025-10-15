/**
 * @file MostPasses.h
 *
 * This file implements the Most Passes leaderboard challenge for RC2025
 * 
 * The basic idea is:
 * - set up ball on corner of penalty area
 * - pass back and forth between two players bringing the ball closer to the
 *   opponent penalty area on each pass
 * - must receive the ball in the opponent penalty area within 180 s of starting
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/InterceptSkills.h"

#include "Tools/TextLogging.h"


namespace CoroBehaviour
{
namespace RC2025
{
namespace Leaderboards
{
  /**
   * Pass a ball to position chosen by the caller
   */
  CRBEHAVIOUR(GotoBallAndPassTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndPassTask) {}

    void operator()(const Vector2f& target)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_BEGIN();

      while (true)
      {
        updateKickParams(target);
        gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely, kickDistance, /* preStepAllowed */ true,
                            /* turnKickAllowed */ false, walkSpeed,
                            /* ignoreObstacles */ true, /* directionPrecision */ Rangea(-2_deg, 2_deg));

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndPassTask,
    {,
      (Rangef)(2000.f, 7000.f) forwardFastLongKickRange, ///< In this range the kick would reach the target
      (Rangef)(0.f, 6500.f) forwardFastKickRange, ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange, ///< In this range the kick would reach the target
      (Angle)(10_deg) minKickSectorSize,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;

    Pose2f walkSpeed;
    int kickSide = 0; // default


    CommonSkills commonSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    void updateKickParams(const Vector2f& target)
    {
      walkSpeed = theFieldBall.positionRelative.norm() < 200.f ? Pose2f(0.7f, 0.7f, 0.7f) : Pose2f(1.f, 1.f, 1.f);

      kickDistance = target.norm() + 500.f; // approximation wtih margin //used to be 5000
      kickAngle = target.angle();

      // Angle ballAngle = theFieldBall.positionRelative().angle();

      // choose the kick type (based on distance and angle)

      // if (kickAngle > 15_deg) // kicking with right foot (toward our left)
      //   targetIsLeft = true;
      // else if (kickAngle < -15_deg) // kicking with left foot (toward our right)
      //   targetIsLeft = false;
      
      if (theFieldBall.positionRelative.y() > 70.f) // kicking with left
        kickSide = 1;
      else if (theFieldBall.positionRelative.y() < -70.f) // kicking with right
        kickSide = -1;
      else if (kickSide == 0)
        kickSide = Random::bernoulli() ? 1 : -1;
      // else leave kickSide at current value

      kickAlignPrecisely = true; // default unless changed below

      // how strong to make the kick?
      if (params.forwardFastKickRange.isInside(kickDistance))
        kickType = (kickSide == 1) ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else
        kickType = (kickSide == 1) ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;

      ACTGRAPH_FMT("kickAngle = {:.1f} deg", toDegrees(kickAngle));
      ACTGRAPH_FMT("distance = {:.0f}", kickDistance);
      ACTGRAPH_FMT("kickType = {}", TypeRegistry::getEnumName(kickType));
    }
  };



  /**
   * a quick turn on the spot search for ball
   * We assume the ball will be fairly close and likely behind use
   */
//   CRBEHAVIOUR(QuickSearchForBallTask)
//   {
//     CRBEHAVIOUR_INIT(QuickSearchForBallTask) {}
// 
//     void operator()()
//     {
//       CR_BEGIN();
// 
//       // turn 180 deg and back up a little
//       chooseInitialSearchPose();
// 
//       CR_CHECKPOINT(turn_to_initial_search_pose);
//       do
//       {
//         updateInitialSearchPose();
//         headSkills.lookAtAngles(0, 23_deg); // to match tilt of look left and right
//         walkToPoseTask(initialSearchPose, 1.f, AVOIDANCE_NONE);
//         CR_YIELD();
//       }
//       while (!CommonSkills::isPoseClose(initialSearchPose, 100.f, 3_deg) && (theFieldBall.timeSinceBallWasSeen > 0));
// 
//       CR_CHECKPOINT(scan_for_ball);
//       while ((getCheckpointDuration() < 4000) && (theFieldBall.timeSinceBallWasSeen > 0))
//       {
//         lookLeftAndRightTask();
//         commonSkills.stand();
//         CR_YIELD();
//       }
// 
//       lookLeftAndRightTask();
//       commonSkills.stand();
// 
//       if (theFieldBall.timeSinceBallWasSeen == 0)
//         CR_EXIT_SUCCESS();
//       else
//         CR_EXIT_FAILED();
//     }
// 
//   private:
//     READS(RobotPose);
//     READS(FieldBall);
// 
//     HeadSkills headSkills {env};
//     CommonSkills commonSkills {env};
//     WalkToPoseTask walkToPoseTask {env};
//     LookLeftAndRightTask lookLeftAndRightTask {env};
// 
// 
//     Pose2f initialSearchPoseOnField;
//     Pose2f initialSearchPose;
// 
//     void chooseInitialSearchPose()
//     {
//       // reverse the robot pose and back up
//       int robotSide = theRobotPose.translation.y() > 0 ? 1 : -1;
// 
//       initialSearchPoseOnField = Pose2f(90_deg * robotSide, theRobotPose.translation + Vector2f(0, -200.f * robotSide));
//     }
// 
//     void updateInitialSearchPose()
//     {
//       initialSearchPose = theRobotPose.toRobotCoordinates(initialSearchPoseOnField);
//     }
//   };

/**
 * standard turn on the spot search
 */
CRBEHAVIOUR(QuickSearchForBallTask)
  {
    CRBEHAVIOUR_INIT(QuickSearchForBallTask) {}

    void operator()(void)
    {
      CR_BEGIN();

      // remember initial pose so we know when we have completed a revolution
      // saveInitialPose();

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
    READS(RobotPose);
    READS(FieldBall);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};

    int turnDirection; // 1 or -1

    void chooseTurnDirection()
    {
      // turn which ever direction will allow robot to face the sideline most quickly
      if ((-90_deg < theRobotPose.rotation) && (theRobotPose.rotation <= 90_deg))
        turnDirection = 1; // anticlockwise
      else
        turnDirection = -1;
    }   
  };


  /**
   * a quick turn on the spot search for ball
   * We assume the ball will be fairly close and likely behind use
   */
  CRBEHAVIOUR(InitialScanForBallTask)
  {
    CRBEHAVIOUR_INIT(InitialScanForBallTask) {}

    void operator()()
    {
      CR_BEGIN();

      if (isScanNeeded())
      {
        TLOGD(tlogger, "InitialScanForBallTask starting");

        while (true)
        {
          lookLeftAndRightTask();
          commonSkills.stand();

          if (theFieldBall.anyBallWasSeen(params.maxTimeSinceBallSeen))
          {
            TLOGD(tlogger, "InitialScanForBallTask success - ball seen");
            CR_EXIT_SUCCESS();
          }
          else if (getCheckpointDuration() >= params.scanDuration)
          {
            TLOGD(tlogger, "InitialScanForBallTask failure - scan timed out");
            CR_EXIT_FAILED();
          }
          else
            CR_YIELD();
        }
      }
    }

    bool isScanNeeded()
    {
      bool anyBallSeen = theFieldBall.anyBallWasSeen(params.maxTimeSinceBallSeen);
      bool closeToStart = closeToStartingPosition(); 
      bool ret = (!anyBallSeen && closeToStart);

      if (!ret)
        TLOGD(tlogger, "isScanNeeded false: anyBallSeen {}, closeToStart {}, timeSinceBallWasSeen {}, timeSinceTeamBallWasValid {}, robotPose {}, startPos {}",
              anyBallSeen, closeToStart,
              theFieldBall.timeSinceBallWasSeen, theFieldBall.timeSinceTeamBallWasValid, theRobotPose,
              Vector2f(theFieldDimensions.xPosOwnPenaltyMark,
                       theFieldDimensions.yPosLeftSideline * ((theRobotPose.translation.y() > 0) ? 1 : -1)));
      return ret;
    }

  private:
    DEFINES_PARAMS(InitialScanForBallTask,
    {,
      (int)(1200) maxTimeSinceBallSeen,
      (unsigned)(1500) scanDuration,
      (float)(400.f) closeToStartDistance,
    });
    
    READS(RobotPose);
    READS(FieldBall);
    READS(BallModel);
    READS(FieldDimensions);
    READS(TeamData);
    READS(TeammatesLocationModel);
    READS(ActiveTactic);
    READS(GameInfo);

    CommonSkills commonSkills {env};
    LookLeftAndRightTask lookLeftAndRightTask {env};

    DECL_TLOGGER_VAR(tlogger, "InitialScanForBallTask", TextLogging::INFO);

    bool closeToStartingPosition()
    {
      int side = (theRobotPose.translation.y() > 0) ? 1 : -1;

      return CommonSkills::arePointsClose(theRobotPose.translation,
          Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftSideline * side),
          params.closeToStartDistance);
    }


  };




  /**
  * The receiver walks to the designated formation pose (possibly modified if needed to deal with opponents).
  * Then the receiver awaits the pass and intercepts as needed.
  */

  CRBEHAVIOUR(MostPassesReceiverPlayingTask)
  {
    CRBEHAVIOUR_INIT(MostPassesReceiverPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      if (initialScanForBallTask.isScanNeeded())
      {
        CR_CHECKPOINT(receiver_initial_scan_for_ball);
        CR_UNTIL(initialScanForBallTask.isEnded(), initialScanForBallTask());
      }

      while (true)
      {
        if (interceptTask.isInterceptNeeded(/* aggressive */ true))
        {
          CR_CHECKPOINT(receiver_intercept_ball);
          CR_UNTIL(interceptTask.isSuccess(), interceptTask(InterceptTask::WALK_INTERCEPTS, /* aggressive */ true));
        }
        else
        {
          CR_CHECKPOINT(receiver_go_to_tactic_pose);
          if (shouldLookAtBall())
            headSkills.lookAtBall();
          else
            headSkills.lookActive(/* withBall: */ true);
          walkToPoseTask(getTacticPose(), params.walkSpeed, AVOIDANCE_NONE, /* keepTargetRotation */ true,
                         /* standAtTarget */ true, params.distanceThresholds, params.angleThresholds);
          CR_YIELD();
        }
      }
    }

  private:
    DEFINES_PARAMS(MostPassesReceiverPlayingTask,
    {,
      (float)(1.f) walkSpeed,
      (HystThresholdsf)(100.f, 150.f) distanceThresholds, // mm
      (HystThresholdsa)(5_deg, 10_deg) angleThresholds,
      (float)(100.f) ballMovingTowardsVelocityX, // mm/s
      (int)(300) lookAtBallSeenTimeout,
    });
    
    READS(RobotPose);
    READS(ActiveTactic);
    READS(BallModel);
    READS(FieldBall);
    READS(FieldDimensions);
    
    HeadSkills headSkills {env};
    InterceptTask interceptTask {env};
    WalkToPoseTask walkToPoseTask {env};
    InitialScanForBallTask initialScanForBallTask {env};

    Pose2f tacticPose; // robot relative

    Pose2f getTacticPose()
    {
      tacticPose = theRobotPose.toRobotCoordinates(theActiveTactic.formationPose);
      return tacticPose;
    }

    bool shouldLookAtBall()
    {
      int robotSide = theRobotPose.translation.y() > 0 ? 1 : -1; // left : right

      bool ballPosMustBeApproaching =
          theFieldBall.ballWasSeen(params.lookAtBallSeenTimeout) &&
          (robotSide == 1 ? theFieldBall.positionOnField.x() >= -theFieldDimensions.centerCircleRadius
                          : theFieldBall.positionOnField.x() <= theFieldDimensions.centerCircleRadius);
      return ballPosMustBeApproaching || (theBallModel.estimate.velocity.x() <= -params.ballMovingTowardsVelocityX);
    }
  };

  // ------------------------------------------------------------------------
  // Ball player behaviour
  // ------------------------------------------------------------------------

  /**
  * Robot 1 should look at the position of defender 2 (the one on the edge of the opposition box).
  * It should pass to the same side of the pitch that the defender 2, in relation to it's y-axis,
  * in order to give more space for the second supporter to have more space for a successful shot. 
  */

  CRBEHAVIOUR(MostPassesBallPlayerPlayingTask)
  {
    CRBEHAVIOUR_INIT(MostPassesBallPlayerPlayingTask) {}

    void operator()()
    {
      CRBEHAVIOUR_BEGIN();

      if (initialScanForBallTask.isScanNeeded())
      {
        TLOGD(tlogger, "initialScanForBall needed");
        CR_CHECKPOINT(ballplayer_initial_scan_for_ball);
        CR_UNTIL(initialScanForBallTask.isEnded(), initialScanForBallTask());
      }
      else
        TLOGD(tlogger, "initialScanForBall not needed");


      // while ball position is not close enough to goal, pass back and forth
      while (!ballInTargetZone())
      {
        // the ball must be on our side (otherwise we wouldn't be the ball player)
        if (!theFieldBall.anyBallWasSeen(params.ballSeenTimeout))
        {
          CR_CHECKPOINT(find_ball_if_needed);          
          CR_UNTIL(quickSearchForBallTask.isSuccess(), quickSearchForBallTask());
        }
        else if (interceptTask.isInterceptNeeded(/* aggressive */ true))
        {
          CR_CHECKPOINT(intercept_needed);
          CR_UNTIL(interceptTask.isSuccess(), interceptTask(InterceptTask::WALK_INTERCEPTS, /* aggressive */ true));
        }
        else if (isPassNeeded())
        {
          if (tooFarToPass() || ballInTargetZone(/* onBallPlayerSide */ true))
          {
            CR_CHECKPOINT(passing_dribble_fwd);
            gotoBallAndDribbleTask(getDribbleDirection(), /* ignoreObstacles */ true, /* kickPower */ 0.1f);
          }
          else if (!teammateNearPosition()) // && (getCheckpointDuration() < params.maxWaitForTeammatePosMs))
          {
            CR_CHECKPOINT(passing_wait_for_receiver_pos);
            // tlogTeammateNearPosition();

            lookAtBallOrLookActive();
            walkToPoseTask(getPrepassPose(), params.walkSpeed, AVOIDANCE_NONE, /* keepTargetRotation */ true,
                          /* standAtTarget */ true);
          }
          else
          {
            CR_CHECKPOINT(passing);
            // TLOGD(tlogger, "pass_to_teammate - passing");

            choosePassTarget();

            commonSkills.recordPassTarget(passTargetPlayer, passTargetOnField);
            gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          }
          CR_YIELD();
        }
        else
        {
          CR_CHECKPOINT(just_stand);
          commonSkills.standLookActive();
          CR_YIELD();
        }
      }

      // wait until higher level behaviour switches out of this coroutine
      CR_CHECKPOINT(ball_inside_target_zone);
      CR_WHILE(true, commonSkills.standLookActive());
    }

  private:
    DEFINES_PARAMS(MostPassesBallPlayerPlayingTask, 
    {, 
      (float)(1.f) walkSpeed, 
      (float)(50.f) minBallMovingVelocity, // mm/s
      (float)(100.f) ballMovingTowardsVelocityX, // mm/s
      (float)(400.f) closeEnoughToPositionForPass, // mm - is the receiver close enough to their position that we can pass now
      (float)(250.f) prepassDistance, // mm - don't forget to allow distance for the foot length
      (unsigned)(4000) maxWaitForTeammatePosMs,
      (int)(1000) ballSeenTimeout,
      (int)(300) lookAtBallSeenTimeout,
      (float)(1200.f) dribbleSafeMargin,
    });

    READS(RobotPose);
    READS(FieldBall);
    READS(BallModel);
    READS(FieldDimensions);
    READS(TeamData);
    READS(TeammatesLocationModel);
    READS(ActiveTactic);
    READS(GameInfo);
    READS(BallSpecification);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
    GotoBallAndPassTask gotoBallAndPassTask{env};
    InterceptTask interceptTask {env};
    WalkToPoseTask walkToPoseTask {env};
    QuickSearchForBallTask quickSearchForBallTask {env};
    InitialScanForBallTask initialScanForBallTask {env};
    GotoBallAndDribbleTask gotoBallAndDribbleTask {env};

    DECL_TLOGGER_VAR(tlogger, "MostPassesBallPlayerPlayingTask", TextLogging::INFO);

    int passTargetPlayer;
    Vector2f passTargetOnField;

    int getRobotSide()
    {
      return theRobotPose.translation.y() > 0 ? 1 : -1; // left : right
    }

    int getTeammateNumber()
    {
      return (theGameInfo.playerNumber == 1) ? 2 : 1;
    }

    bool tooFarToPass()
    {
      const float formationY =
          theActiveTactic.formationPoses[Settings::toPlayerIndex(theGameInfo.playerNumber)].translation.y();
      const float passingZoneY = std::abs(formationY) + 500.f;

      const bool ballInDribbleSafeZone = (std::abs(theFieldBall.endPositionOnField.y()) >
                                          (theFieldDimensions.centerCircleRadius + params.dribbleSafeMargin));
      const bool ballOutsidePassingZone = (std::abs(theFieldBall.endPositionOnField.y()) > passingZoneY);
      bool ret = ballInDribbleSafeZone && ballOutsidePassingZone;

      return ret;
    }

    Angle getDribbleDirection() //robot relative
    {
      // get angle from ball to formationPose
      Vector2f ballToFormationPos =
          theActiveTactic.formationPoses[Settings::toPlayerIndex(theGameInfo.playerNumber)].translation -
          theFieldBall.endPositionOnField;
      return theRobotPose.toRobotCoordinates(ballToFormationPos).angle();
    }

    void lookAtBallOrLookActive()
    {
      if (shouldLookAtBall())
        headSkills.lookAtBall();
      else
        headSkills.lookActive(/* withBall: */ true);
    }

    bool shouldLookAtBall()
    {
      bool ballPosMustBeApproaching =
          theFieldBall.ballWasSeen(params.lookAtBallSeenTimeout) &&
          (getRobotSide() == 1 ? theFieldBall.positionOnField.x() >= -theFieldDimensions.centerCircleRadius
                               : theFieldBall.positionOnField.x() <= theFieldDimensions.centerCircleRadius);
      return ballPosMustBeApproaching || (theBallModel.estimate.velocity.x() <= -params.ballMovingTowardsVelocityX);
    }

    /** check if ball is in the target zone in either the ball players or receivers side
     * assuming that the current robot (theRobotPose) is always the ball player
     */
    bool ballInTargetZone(bool onBallPlayerSide = false)
    {
      int ballPlayerSide = theRobotPose.translation.y() > 0 ? 1 : -1;
      int desiredPlayerSide = onBallPlayerSide ? ballPlayerSide : -ballPlayerSide;
      int ballSide = theFieldBall.positionOnField.y() > 0 ? 1 : -1;

      bool ret = (theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea) &&
                 (desiredPlayerSide == ballSide) &&
                 (std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftPenaltyArea);

      // TLOGD(tlogger,
      //       "ballInTargetZone onBallPlayerSide {}, insidePenX {}, ballSide==playerSide {}, insidePenY {}, return {}",
      //       onBallPlayerSide, (theFieldBall.positionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea),
      //       (desiredPlayerSide == ballSide),
      //       (std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftPenaltyArea), ret);

      return ret;
    }

    bool ballMovingAcross()
    {
      Vector2f ballVelocityOnField = theBallModel.estimate.velocity.rotated(theRobotPose.rotation);

      // ballPlayer kicking left to right:
      // - robot y is positive, ballVelocity.y is negative, theFieldBall.endPositionOnField.y is negative
      // ballPlayer kicking right to left
      // - robot y is negative, ballVelocity.y is positive, theFieldBall.endPositionOnField.y is positive

      int robotSide = getRobotSide();

      return ((theFieldBall.endPositionOnField.y() * -robotSide) > 0) ||
             ((ballVelocityOnField.y() * -robotSide) > params.minBallMovingVelocity);
    }

    bool isPassNeeded()
    {
      if (theRobotPose.translation.y() > 0) // is robot on the left
        return theFieldBall.recentBallEndPositionOnField().y() > theFieldDimensions.centerCircleRadius; // ball still outside L excl zone?
      else // robot is on right
        return theFieldBall.recentBallEndPositionOnField().y() < -theFieldDimensions.centerCircleRadius; // ball still outside R excl zone?
    }

    Vector2f getTeammateLocation()
    {
      int teammateNumber = getTeammateNumber();
      for (auto& location : theTeammatesLocationModel.locations)
      {
        if (location.playerNumber == teammateNumber)
          return location.pose.translation;
      }

      return Vector2f(99999.f,99999.f);
    }

    bool teammateNearPosition()
    {
      int teammateIndex = Settings::toPlayerIndex(getTeammateNumber());

      // check if teammate's estimated location is near their intended tactic/formation position
      // get estimated location
      return commonSkills.arePointsClose(getTeammateLocation(),
                                         theActiveTactic.formationPoses[teammateIndex].translation,
                                         params.closeEnoughToPositionForPass);
    }

    void tlogTeammateNearPosition()
    {
      Vector2f teammateLocation = getTeammateLocation();
      Vector2f teammateFormationPos =
          theActiveTactic.formationPoses[Settings::toPlayerIndex(getTeammateNumber())].translation;
      bool isClose =
          commonSkills.arePointsClose(teammateLocation, teammateFormationPos, params.closeEnoughToPositionForPass);

      TLOGD(tlogger, "teammateNearPosition {}: teammate #{} {}, pos {}, dist {:.1f}", isClose, getTeammateNumber(),
            teammateLocation, teammateFormationPos, (teammateFormationPos - teammateLocation).norm());
    }

    /// get a pose near the ball and near the kick pose required for making the
    /// the pass, so that we can wait there if necessary
    Pose2f getPrepassPose()
    {
      Vector2f teammatePosOnField =
          theActiveTactic.formationPoses[Settings::toPlayerIndex(getTeammateNumber())].translation;

      Vector2f ballPos = theFieldBall.recentBallEndPositionOnField();
      Angle targetAngleFromBall = (teammatePosOnField - ballPos).angle();
      Pose2f prepassPoseOnField = Pose2f(targetAngleFromBall, ballPos);
      prepassPoseOnField.translate(Vector2f(-params.prepassDistance, 0)); // back up from the ball

      Pose2f prepassPose = theRobotPose.toRobotCoordinates(prepassPoseOnField);

      // // check if walkig to the pre
      // // FIXME HACK if prepassPose is close to side and behind us, then backup first
      // Geometry::Circle ball(theFieldBall.recentEndPositionOnField, theBallSpecification.radius);
      // if ((std::abs(prepassPose.translation.y()) < 200.f) && (std::abs(prepassPose.translation.angle()) > 100_deg))
      //   prepassPose.translation.y() += (prepassPose.translation.y() > 0) ? -200.f : 200.f;

      // will walk to the ball hit the ball
//       Vector2f intersection1, intersection2;
//       const Vector2f startPoint = Vector2f(0,0);
//       int intersections = Geometry::getIntersectionOfLineAndCircle(
//           Geometry::Line(startPoint, prepassPose.translation.normalized()),
//           Geometry::Circle(theFieldBall.recentBallEndPositionRelative(), theBallSpecification.radius + 50.f),
//           intersection1, intersection2);
// 
//       if (intersections > 0)
//       {
//         // find out which rotation for walking sideways makes the most sense and walk sideways, if the needed adjustment in the rotation is small enough (45deg)
//         const Angle sideOne = Angle::normalize(90_deg - prepassPose.translation.angle());
//         const Angle sideTwo = Angle::normalize(-90_deg - prepassPose.translation.angle());
// 
//       }

      return prepassPose;
    }

    void choosePassTarget()
    {
      passTargetPlayer = getTeammateNumber();

      // get position of pass target
      // For now we'll assume that communication messages are sent often
      // enough that we can use the directly communicated position

      // default position if no comms (our current x position plus a bit and the opposite y position)
      // passTargetOnField.x() = theRobotPose.translation.x() + 300.f;
      // passTargetOnField.y() = theRobotPose.translation.y() > 0 ? theFieldDimensions.yPosRightPenaltyArea
      //                                                          : theFieldDimensions.yPosLeftPenaltyArea;

      int teammateIndex = Settings::toPlayerIndex(passTargetPlayer);
      passTargetOnField = theActiveTactic.formationPoses[teammateIndex].translation;

      // FIXME HACK - passes seem to regularly fall behind the target robot
      if (passTargetOnField.x() > (theRobotPose.translation.x() + 200.f))
        passTargetOnField.x() += 150.f;

      // for (const Teammate& teammate : theTeamData.teammates)
      // {
      //   if (teammate.number == passTargetPlayer)
      //   {
      //     passTargetOnField = teammate.theRobotPose.translation;
      //     passTargetOnField.x() += 500.f; // pass slightly ahead of the player
      //     passTargetOnField.y();
      //     return;
      //   }
      // }
    }
  };

  /**
   * This is the entry point task for the playing state. It decides which
   * specialized behaviour to run
   */
  CRBEHAVIOUR(MostPassesPlayingStateTask)
  {
    CRBEHAVIOUR_INIT(MostPassesPlayingStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if ((theGameInfo.playerNumber == 1) || (theGameInfo.playerNumber == 2))
        {
          if (theActiveTactic.situationRole == PlayerRole::ballPlayer)
            ballPlayerPlayingTask();
          else
            receiverPlayingTask();
        }

        CR_YIELD();
      }
    }

  private:
    READS(ActiveTactic);
    READS(GameInfo);

    MostPassesBallPlayerPlayingTask ballPlayerPlayingTask {env};
    MostPassesReceiverPlayingTask receiverPlayingTask {env};
  };

} // Leaderboards
} // RC2025
} // CoroBehaviour
