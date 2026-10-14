/**
 * @file GotoSkills.h
 *
 * This file contains some RE2019 based goto skills.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour
 * engine.
 *
 * Note: K1 kicks are not handled here. GotoBallAndKickTask remaps every kick
 * type chosen in this file to an RL kick; only the kick foot choice survives.
 *
 * @author Rudi Villing
 * @author James Petri
 * @author Andy Lee Mitchell
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/BallSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/ObstacleSkills.h"

#include "BallAndKickSkills.h"

#include "TextLogging/TextLogging.h"


namespace RE
{
namespace v2025
{

  // =====================================================================

  /**
   * Update the kick foot choice: extreme kick angles force the foot,
   * otherwise hysteresis on ball y keeps the foot nearest the ball.
   * On K1 this foot choice is the only thing that survives from the kick
   * types chosen in this file (GotoBallAndKickTask remaps them to RL kicks).
   */
  inline bool updateKickFoot(Comparatorf& kickToLeft, Angle kickAngle, float ballYRelative)
  {
    if (kickAngle < -45_deg) // aiming right => left foot
      kickToLeft.state = true;
    else if (kickAngle > 45_deg) // aiming left => right foot
      kickToLeft.state = false;
    else
      kickToLeft.update(ballYRelative);

    return kickToLeft.state;
  }

  // =====================================================================

  /**
   * Main entry point of the striker behaviour in playing state
   */
  CRBEHAVIOUR(GotoBallAndKickBestOptionTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndKickBestOptionTask) {}

    void operator()(bool clearance = false)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_BEGIN();

      passing.resetThresholds(params.passXThresholds);
      // kickToLeft.resetThresholds(params.kickAngleThresholds);
      kickToLeft.resetThresholds(params.kickFootThresholds);
      ballWasKickable = false; // reset each time we restart this skill from the top (but not every time we resume after a yield)
      // passingNow = false; // will be modified if we are passing

      while (true)
      {
        // try not to walk into goal
        if (theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField))
        {
          CR_CHECKPOINT(stand_because_ball_in_goal);
          headSkills.lookActive(/* withBall */ true, /* ignoreBall */ false, /* onlyOwnBall */ true);
          commonSkills.stand();
          CR_EXIT_SUCCESS();
        }

        // enable/disable duelling delay via the behaviourParams.demoDuellingDelay
        if (checkDuellingDelayRequired())
        {
          CR_CHECKPOINT(duel_delay_active);
          headSkills.lookActive(/* withBall */ true, /* ignoreBall */ false, /* onlyOwnBall */ true);
          motionSkills.walkAtRelativeSpeed(Pose2f(0.f, 0.f, 0.f)); // walk on the spot
        }
        else
        {
          // if (needsClearanceKick() || clearance)
          if (clearance)
          {
            CR_CHECKPOINT(proceed_to_kick_clearance);
            setClearanceKickAngleAndDistance();
            setClearanceKickType();
            gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);
          }
          // ANDY COMMENT THIS OUT TO REMOVE PASSING
          // else if (theFieldBall.endPositionOnField.x() < 1500.f &&
          //           theFieldBall.endPositionOnField.x() > theFieldDimensions.xPosHalfwayLine)
          // else if ((theRETeamData.countActiveTeammatesKickedSinceRestart() == 0) &&
          //          passing.update(theFieldBall.endPositionOnField.x()))
          // //  (theFieldBall.endPositionOnField.x() >
          // //   (wasPassing ? theFieldDimensions.xPosHalfwayLine - 250.f : theFieldDimensions.xPosHalfwayLine - 100)))
          // {
          //   CR_CHECKPOINT(proceed_to_pass);
          //   setPassAngleAndDistance();
          //   setPassKickType();
          //   gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);
          //   // passingNow = true;
          // }
          else
          {
            CR_CHECKPOINT(proceed_to_kick);
            updateKickParams();
            gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);
          }
        }

        // wasPassing = passingNow; // remember for hysteresis next time

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndKickBestOptionTask,
    {,
      // moved to REBehaviourParams
      // (Rangef)(2000.f, 6000.f) forwardFastLongKickRange,  ///< In this range the kick would reach the target
      // (Rangef)(1600.f, 2500.f) forwardFastKickRange,      ///< In this range the kick would reach the target
      // (Rangef)(800.f, 2000.f) walkForwardsLongKickRange,  ///< In this range the kick would reach the target
      // (Rangef)(0.f, 1000.f) walkForwardsKickRange,        ///< In this range the kick would reach the target
      (Angle)(10_deg) minKickSectorSize,
      (float)(500.f) duelObstacleDistance,
      (float)(400.f) duelBallDistance,
      (Vector2f)(100.f, 100.f) ballKickableDistance, ///< when the ball is closer than this to front/side of the feet, we assume we can kick
      (Vector2f)(150.f, 150.f) ballNotKickableDistance, ///< when the ball is farther than this from front/side of the feet, we assume we can no longer kick
      (float)(75.f) duellingBallMovedDistance, ///< when the ball moves by this amount in a duel, we assume an opponent moved it
      // (Angle)(15_deg) switchKickThreshold,  // hysteresis value for switching which foot we kick with when the kickAngle changes
      // (HystThresholdsa)(-15_deg, 15_deg) kickAngleThresholds, ///< kick left if greater than upper, right if less than lower
      (HystThresholdsf)(-70, 70) kickFootThresholds,
      (Vector2f)(1600.f, 1500.f) kickAwayMax, ///< we'll go to the ball and kick it away at these distances from the ground line and y=0 line
      // (float)(1600.f) kickAwayXMax, ///< we will go to the ball and kick it away in these x distances from ground line
      // (float)(1500.f) kickAwayYMax, ///< maximum y-value at which we kick away. Together With kickAwayXRange, this defines a box
      (unsigned)(3000) kickAwayBallSeenTimeoutMs, ///< we'll allow the ball to be obstructed briefly on the way to kicking it away
      (HystThresholdsf)(-250.f, -100.f) passXThresholds, // we'll pass if x > highThresh or not if x < low threshold
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameState);
    READS(REBehaviourParams);
    READS(FrameInfo);
    READS(RobotDimensions);
    READS(RETeamData);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    KickPrecision kickAlignPrecisely;
    Vector2f duellingObstacleCenter;
    unsigned timeWhenDuelDelayStarted = 0;
    bool ballWasKickable = false;
    Vector2f prevEndPositionOnField; // 0,0
    // bool kickToLeft = false;
    // bool passingNow = false;
    // bool wasPassing = false;
    Comparatorf passing { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ true };
    // Comparatora kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ false };
    Comparatorf kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ false };

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};



    bool checkDuellingDelayRequired()
    {
      ACTGRAPH_FMT("ballWasKickable: {},\nballPos: {}", ballWasKickable, theFieldBall.positionRelative);

      if (theREBehaviourParams.demoDuellingDelayMs == 0)
        return false;

      // are we already in a delay
      if (timeWhenDuelDelayStarted &&
          (theFrameInfo.getTimeSince(timeWhenDuelDelayStarted) < theREBehaviourParams.demoDuellingDelayMs))
        return true;

      // if we're not already delaying, starting a new delay may be required
      // if the ball has moved since the last time we delayed (probably because
      // an opponent touched it)

      bool delayRequired = false;

      if (checkDuelling())
      {
        if ((theFieldBall.endPositionOnField - prevEndPositionOnField).norm() > params.duellingBallMovedDistance)
        {
          timeWhenDuelDelayStarted = theFrameInfo.time;
          delayRequired = true;
        }

        // update the prevPosition on every cycle where the ball hasn't moved (much)
        // or at the start of a duelling delay
        prevEndPositionOnField = theFieldBall.endPositionOnField;

//         const float frontOfFoot = theRobotDimensions.footLength;
//         const float sideOfFoot = theRobotDimensions.yHipOffset + theRobotDimensions.soleToOuterEdgeLength;
// 
//         // was it kickable before and not kickable now => it was just stolen from us
//         if (ballWasKickable &&
//             ((theFieldBall.positionRelative.x() < 0) ||
//              (theFieldBall.positionRelative.x() > (frontOfFoot + params.ballNotKickableDistance.x())) ||
//              (std::fabs(theFieldBall.positionRelative.y()) > (sideOfFoot + params.ballNotKickableDistance.y()))))
//         {
//           // potentially delay if we are a weak team
//           timeWhenDuelDelayStarted = theFrameInfo.time;
//           ballWasKickable = false; // remember for next time
//           return true; // yep - delay
//         }
//         else if (!ballWasKickable && (theFieldBall.positionRelative.x() > 0) &&
//                  (theFieldBall.positionRelative.x() <= (frontOfFoot + params.ballKickableDistance.x())) &&
//                  (std::fabs(theFieldBall.positionRelative.y()) <= (sideOfFoot + params.ballKickableDistance.y())))
//         {
//           ballWasKickable = true; // remember for next time
//           return false;
//         }
      }

      return delayRequired;
    }

    // A duel is taking place if an opponent is closer than threshold to the ball
    // and we are within duel distance from the ball
    bool checkDuelling()
    {
      // if there are no obstacles there is no duel
      if (theObstacleModel.obstacles.empty())
        return false;

      // find the robot/obstacle closest to the ball
      float closestDistance = std::numeric_limits<float>::max();

      for(const Obstacle& o : theObstacleModel.obstacles)
      {
        // FIXME - no goalpost detection any more?
        // if(o.type != Obstacle::Type::goalpost) // && o.type != Obstacle::Type::unknown)
        {
          const float sqrNorm = (o.center - theFieldBall.positionRelative).squaredNorm();

          if(sqrNorm < closestDistance)
          {
            closestDistance = sqrNorm;
            duellingObstacleCenter = o.center;
          }
        }
      }

      // is the closest robot to the ball a threat such that we are duelling
      const float closestObstacleDistanceToBall = std::sqrt(closestDistance);
      const float distanceToBall = theFieldBall.positionRelative.norm();

      bool duelling = (closestObstacleDistanceToBall < params.duelBallDistance) &&
                      (distanceToBall < params.duelBallDistance);

      return duelling;
    }

    void updateKickParams()
    {
      if (isKickTowardsGoalBestOption())
      {
        setGoalKickAngleAndDistance();
        setGoalKickType();
        ACTGRAPH_FMT("kickTowardsGoal angle={}", kickAngle);
      }
      else
      {
        setDuelKickAngle();
        setDuelKickType();
        ACTGRAPH_FMT("duelling angle={}", kickAngle);
      }
      bool checkForward = isKickAngleForward(kickAngle);
      ACTGRAPH_FMT("kickAngleIsForward: {}", checkForward);
    }
    
    bool isKickTowardsGoalBestOption()
    {
      // TODO: remove me to enable proper duel behaviour once working correctly.
      return true;

      bool decision = false;
      
      // if we are in the opponent box always try and shoot
      if (theRobotPose.translation.x() > theFieldDimensions.xPosOpponentPenaltyArea - 1000.f)
      {
        decision = true;
        return decision;
      }
      // we never want to duel as a goalie
      if (theGameState.isGoalkeeper())
      {
        decision = true;
        return true;
      }
      // is the opponent close enough to be a danger (i.e. do we need to duel?)
      if (obstacleSkills.isOpponentDangerNearBallDuel(800.f))
      {
        ACTGRAPH_FMT("Duelling - opponent close to ball!");
        decision = false;
      }
      else
      {
        ACTGRAPH_FMT("No opponent close to ball");
        decision = true;
      }
      // is the nearest opponent behind us on the pitch?
      // if (!isKickAngleForward(nearestOpponentAngle, 100.f))
      // {
      //   ACTGRAPH_FMT("shooting time");
      //   decision = true;
      // }
      return decision;
    }

    void setClearanceKickAngleAndDistance()
    {
      Vector2f leftHalfwayLine {theFieldDimensions.xPosHalfwayLine, theFieldDimensions.yPosLeftTouchline};
      Vector2f rightHalfwayLine {theFieldDimensions.xPosHalfwayLine, theFieldDimensions.yPosRightTouchline};
      Vector2f centreHalfwayLine {theFieldDimensions.xPosHalfwayLine, 0.f};
      Rangea bestKickSector;
      bool left = theFieldBall.endPositionOnField.y() > 0.f ? true : false;
      if (theGameState.isGoalkeeper())
      {
        if (left)
        {
          const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(leftHalfwayLine, centreHalfwayLine);
          (void)kickSectors;
          bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);
        }
        else
        {
          const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(centreHalfwayLine, rightHalfwayLine);
          (void)kickSectors;
          bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);
        }
      }
      else
      {
        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(leftHalfwayLine, rightHalfwayLine);
        (void)kickSectors;
        bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);
      }



      if (bestKickSector.getSize() > params.minKickSectorSize)
      {
        kickAngle = (bestKickSector.max + bestKickSector.min) / 2; // mid angle = average
        kickDistance = 4000.f;
      }
      else if (bestKickSector.getSize() > params.minKickSectorSize / 2) // FIXME hack to make sure we don't end up flip-flopping too much
      {
        kickAngle = (bestKickSector.max + bestKickSector.min) / 2; // mid angle = average
        kickDistance = 4000.f;
      }
      else
      {
        if (left)
          kickAngle = theRobotPose.toRobotCoordinates(leftHalfwayLine).angle();
        else
          kickAngle = theRobotPose.toRobotCoordinates(rightHalfwayLine).angle();
        kickDistance = 4000.f;
      }

      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
    }


    void setClearanceKickType()
    {
      ACTGRAPH_FMT("kickAngleDeg: {}", kickAngle);
      // was:
      // if (kickAngle < -45_deg) // Right foot aiming left
      // {
      //   kickToLeft.state = true;
      // }
      // else if (kickAngle > 45_deg) // Left foot aiming right
      // {
      //   kickToLeft.state = false;
      // }
      // else
      // {
      //   kickToLeft.update(theFieldBall.positionRelative.y());
      // }
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickAlignPrecisely = KickPrecision::notPrecise;

      kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
    }


    /// angle to goal in robot relative coordinates
    void setGoalKickAngleAndDistance()
    {
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGoalLine, 0.f};
      Vector2f goalCenter = theRobotPose.toRobotCoordinates(goalCenterOnField);

      kickAngle = goalCenter.angle();
      kickDistance = goalCenter.norm() + 1000.f; // approximation wtih margin

      if (goalCenter.norm() < 6000.f) // FIXME HACK
      {
        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
        (void)kickSectors;
        Rangea bestGoalSector = obstacleSkills.getBestGoalSector(kickSectors);

        if (bestGoalSector.getSize() > params.minKickSectorSize)
        {
          kickAngle = (bestGoalSector.max + bestGoalSector.min) / 2; // mid angle = average
          kickDistance = goalCenter.norm() + 1000.f; // approximation wtih margin
        }
      }

      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
    }

    void setGoalKickType()
    {
      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
      // if (kickAngle < -45_deg) // Right foot aiming left
      // {
      //   kickToLeft.state = true;
      // }
      // else if (kickAngle > 45_deg) // Left foot aiming right
      // {
      //   kickToLeft.state = false;
      // }
      // else
      // {
      //   kickToLeft.update(theFieldBall.positionRelative.y());
      // }
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickAlignPrecisely = KickPrecision::notPrecise;

      bool canTakeLongShot = (theRobotPose.translation.x() >= theREBehaviourParams.longShotAtGoalMinX);

      // kick type by range is Nao only: on K1 all types below collapse to the same RL kick
      if (obstacleSkills.isOpponentDangerNearBallDuel(500.f))
      {
        // if (theRobotPose.translation.x() > theFieldDimensions.centerCircleRadius)
        if (canTakeLongShot && (theREBehaviourParams.walkForwardsLongKickAtGoalRange.getSize() > 0))
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        else
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      }
      else if (canTakeLongShot && theREBehaviourParams.forwardFastLongKickAtGoalRange.isInside(kickDistance))
      {
        kickAlignPrecisely = KickPrecision::precise;
        kickType = kickToLeft.state ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      }
      else if (canTakeLongShot && theREBehaviourParams.forwardFastKickAtGoalRange.isInside(kickDistance))
        kickType = kickToLeft.state ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else if (canTakeLongShot && theREBehaviourParams.walkForwardsLongKickAtGoalRange.isInside(kickDistance))
        kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      else if (theGameState.isGoalkeeper()) // clearance kick
        kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      else
      {
        // prefer an enabled walk kick; if none is configured, fall back instead of aborting
        // (the kick type doesn't matter on K1 anyway)
        if (theREBehaviourParams.walkForwardsKickAtGoalRange.getSize() > 0)
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
        else if (theREBehaviourParams.walkForwardsLongKickAtGoalRange.getSize() > 0)
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        else
        {
          // was: TLOG_ABORT(TextLogging::get("GotoBallAndKickBestOption"),
          //                 "At least one walk kick must have a valid range in behaviourParams.cfg");
          TLOGE(TextLogging::get("GotoBallAndKickBestOption"),
                "no walk kick range configured in behaviourParams.cfg, defaulting to walkForwards");
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
        }
      }

      ACTGRAPH_FMT("kickToLeft: {}\nkickType: {}\nkickAlignPrecisely: {}", kickToLeft.state,
                      TypeRegistry::getEnumName(kickType), TypeRegistry::getEnumName(kickAlignPrecisely));
    }

    void setDuelKickAngle()
    {
      kickDistance = 10.f;
      Angle nearestOpponentAngle = obstacleSkills.getNearestOpponentAngle();

      if (theRobotPose.translation.y() >= 0.f)
      {
        kickAngle = nearestOpponentAngle - 45_deg;
        if (!isKickAngleForward(kickAngle)) // make sure we don't kick towards our own goal!!!
          kickAngle = nearestOpponentAngle + 45_deg;
      }
      else
      {
        kickAngle = nearestOpponentAngle + 45_deg;
        if (!isKickAngleForward(kickAngle)) // make sure we don't kick towards our own goal!!!
          kickAngle = nearestOpponentAngle - 45_deg;
      }
      // clamp kick angle
      if (kickAngle > pi)
        kickAngle = pi;
      else if (kickAngle < -pi)
        kickAngle = -pi;

      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
      // check what direction the kick is (forward or not)
      // addActivationGraphOutput(fmt::format("nearestOpponent distance = {}mm, angle = {}", nearestOpponent.center.norm(), nearestOpponent.center.angle()));
      // addActivationGraphOutput(fmt::format("angle on field = {}", theRobotPose.toFieldCoordinates(Pose2f(kickAngle)).rotation));
    }

    void setDuelKickType()
    {
      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
      // was:
      // if (kickAngle < -45_deg) // Right foot aiming left
      // {
      //   kickToLeft.state = true;
      // }
      // else if (kickAngle > 45_deg) // Left foot aiming right
      // {
      //   kickToLeft.state = false;
      // }
      // else
      // {
      //   kickToLeft.update(theFieldBall.positionRelative.y());
      // }
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      // we want the kick to be as fast as possible
      kickAlignPrecisely = KickPrecision::notPrecise;
      kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
    }

    void setPassAngleAndDistance()
    {
      Vector2f leftPassingLimit {theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosLeftTouchline};
      Vector2f rightPassingLimit {theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosRightTouchline};

      // we want to kick into free area towards the goal (or either side if in the middle of the field)
      Vector2f leftKickLimit;
      Vector2f rightKickLimit;
      if ((std::abs(theFieldBall.endPositionOnField.y()) > theFieldDimensions.centerCircleRadius))
      {
        bool left = theFieldBall.endPositionOnField.y() > 0.f ? true : false;
        if (left)
        {
          leftKickLimit = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosLeftGoalArea);
          rightKickLimit = Vector2f(theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosRightTouchline);
        }
        else
        {
          leftKickLimit = Vector2f(theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosLeftTouchline);
          rightKickLimit = Vector2f(theFieldDimensions.xPosOpponentGoalLine, theFieldDimensions.yPosRightGoalArea);
        }
      }
      else
      {
        leftKickLimit = leftPassingLimit;
        rightKickLimit = rightPassingLimit;
      }

      Rangea bestKickSector;

      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(leftKickLimit, rightKickLimit);
      (void)kickSectors;
      bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);

      // find a free sector and kick into that
      if (bestKickSector.getSize() > 5_deg)
      {
        kickAngle = (bestKickSector.max + bestKickSector.min) / 2; // mid angle = average
        kickDistance = 2000.f;
      }

      // kick ahead of the chosen angle 
      if (isKickAngleLeft(kickAngle))
      {
        kickAngle = kickAngle - 10_deg;
      }
      else
      {
        kickAngle = kickAngle + 10_deg;
      }
      // clamp kick angle
      if (kickAngle > pi)
        kickAngle = pi;
      else if (kickAngle < -pi)
        kickAngle = -pi;
      if (!isKickAngleForward(kickAngle))
      {
        Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGoalLine, 0.f};
        Vector2f goalCenter = theRobotPose.toRobotCoordinates(goalCenterOnField);

        kickAngle = goalCenter.angle();
        kickDistance = goalCenter.norm() + 1000.f; // approximation wtih margin
        ACTGRAPH_FMT("PASS: kickAngle is not forward!");
      }

      // handle close to goal
      if (theFieldBall.endPositionOnField.x() > theFieldDimensions.xPosOpponentPenaltyArea)
      {
        float left = theFieldBall.endPositionOnField.y() > 0.f ? 1.f : -1.f;
        Vector2f target {theFieldBall.endPositionOnField.x(), -theFieldBall.endPositionOnField.y()};
        Vector2f leftPassingLimit {theFieldBall.endPositionOnField.x() + (left*200.f), (left*theFieldDimensions.yPosRightTouchline)};
        Vector2f rightPassingLimit {theFieldBall.endPositionOnField.x() - (left*200.f), (left*theFieldDimensions.yPosRightTouchline)};

        Rangea bestKickSector;

        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(leftKickLimit, rightKickLimit);
        (void)kickSectors;
        bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);

        // if (bestKickSector.getSize() > 1_deg)
        // {
        //   kickAngle = (bestKickSector.max + bestKickSector.min) / 2; // mid angle = average
        //   kickDistance = 1200.f;
        // }

        Vector2f targetRelative = theRobotPose.toRobotCoordinates(target);
        kickAngle = targetRelative.angle();
        kickDistance = 2*std::abs(target.y());
      }

      ACTGRAPH_FMT("PASS: kickDistance: {}", kickDistance);
      ACTGRAPH_FMT("PASS: kickAngle = {}", kickAngle);
    }

    void setPassKickType()
    {
      ACTGRAPH_FMT("kickAngle: {}", kickAngle);
      // was:
      // if (kickAngle < -45_deg) // Right foot aiming left
      // {
      //   kickToLeft.state = true;
      // }
      // else if (kickAngle > 45_deg) // Left foot aiming right
      // {
      //   kickToLeft.state = false;
      // }
      // else
      // {
      //   kickToLeft.update(theFieldBall.positionRelative.y());
      // }
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickAlignPrecisely = KickPrecision::notPrecise;

      kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
    }

    /**
     * Checks if the duel kick angle is towards the opponents side of the field. 
     * This is to prevent us doing a duel kick towards our own goal.
     * @param testKickAngle: the angle to test (in robot relative co-ords)
     * @param limit:         the maximum angle that the angle can be (pos + neg)
    */
    bool isKickAngleForward(Angle testKickAngle, float limit = 80.f)
    {
      Pose2f tempFieldPose =  theRobotPose.toFieldCoordinates(Pose2f(testKickAngle, 0.f, 0.f));
      Angle testKickAngleOnField = tempFieldPose.rotation;
      // addActivationGraphOutput(fmt::format("robot-relative angle = {}, field-relative angle = {}", testKickAngle, testKickAngleOnField));
      return (std::abs(testKickAngleOnField.toDegrees()) < limit);
    }

    bool isKickAngleLeft(Angle testKickAngle)
    {
      Pose2f tempFieldPose =  theRobotPose.toFieldCoordinates(Pose2f(testKickAngle, 0.f, 0.f));
      Angle testKickAngleOnField = tempFieldPose.rotation;
      // addActivationGraphOutput(fmt::format("robot-relative angle = {}, field-relative angle = {}", testKickAngle, testKickAngleOnField));
      return (testKickAngleOnField > 0_deg);
    }

//     bool needsClearanceKick()
//     {
//       // 
//       //if (theREGameInfo.setPlay == SET_PLAY_GOAL_KICK)
//       //  return true;
// 
//       // TODO: the current implementation is pretty naive and just looks at the
//       // ball position. It would be better to consider things like, where
//       // are the opponent players, where are our defensive players, before
//       // committing to go 1v1
// 
//       // NOTE: that we also assume that if the goalie is the closest robot
//       // to the ball it will role switch to become the ball player, in which
//       // case that behaviour will run (and kick the ball) rather than this one
// 
//       Vector2f ballPos = theFieldBall.endPositionOnField;
// 
//       float xThreshold = std::min(theFieldDimensions.xPosOwnGoalLine + params.kickAwayXMax,
//                                   theFieldDimensions.xPosOwnGoalLine + 1700.f);
// 
//       // is the ball in the defensive zone of the field
//       bool inClearanceXRange = (ballPos.x() < xThreshold);
// 
//       // is the ball within the kickAway y range
//       bool inClearanceYRange = (fabs(ballPos.y()) <= params.kickAwayYMax);
// 
//       return theFieldBall.ballWasSeen(params.kickAwayBallSeenTimeoutMs) && inClearanceXRange && inClearanceYRange;
//     }
  };



  CRBEHAVIOUR(GotoBallAndShootFromKickOffTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndShootFromKickOffTask) {}

    void operator()(Angle minKickSectorSize = 10_deg)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      kickToLeft.resetThresholds(params.kickFootThresholds);

      CRBEHAVIOUR_LOOP()
      {
        updateKickParams(minKickSectorSize);
        gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndShootFromKickOffTask,
    {,
      // (Rangef)(2000.f, 4000.f) forwardFastLongKickRange, ///< In this range the kick would reach the target
      // (Rangef)(1500.f, 2500.f) forwardFastKickRange, ///< In this range the kick would reach the target
      // (Rangef)(1000.f, 2000.f) walkForwardsLongKickRange, ///< In this range the kick would reach the target
      // (Rangef)(0.f, 1500.f) walkForwardsKickRange, ///< In this range the kick would reach the target
      (Rangef)(2000.f, 5000.f) forwardFastLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(1600.f, 2500.f) forwardFastKickRange,      ///< In this range the kick would reach the target
      (Rangef)(800.f, 2000.f) walkForwardsLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange,        ///< In this range the kick would reach the target
      (float)(750.f)  exclusionRadius,             ///< Radius around the ball for a legal kick in
      (float)(100.f)  exclusionThreshold,          ///< A 10cm tolerance for the exclusion radius
      (float)(1000.f) upfieldObstacleFreeShotDist, ///< The distance we shoot at if we find a sector with no obstacles (1m)
      (float)(2000.f) maxKickInDistanceNonGoalie,  ///< Maximum distance we are allowing the non goalie robots to shoot at in a kickIn
      (float)(5000.f) goalKickInLongShot,          ///< Distance the goalie shoots upfield if it finds an obstacle free path
      // (Angle)(5_deg) minKickSectorSize,
      (HystThresholdsf)(-70, 70) kickFootThresholds,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameState);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    KickPrecision kickAlignPrecisely;
    Comparatorf kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ true };

    CommonSkills commonSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    void updateKickParams(Angle minKickSectorSize)
    {
      setGoalKickAngleAndDistance(minKickSectorSize);
      setGoalKickType();
      ACTGRAPH_FMT("kickAngle = {}\nkickDistance = {}", kickAngle, kickDistance);
    }

    void setGoalKickAngleAndDistance(Angle minKickSectorSize)
    {
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGoalLine, 0.f};
      Vector2f goalCenter = theRobotPose.toRobotCoordinates(goalCenterOnField);

      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
      (void)kickSectors;
      Rangea bestGoalSector = obstacleSkills.getBestGoalSector(kickSectors);

      if (bestGoalSector.getSize() > minKickSectorSize)
      {
        kickAngle = (bestGoalSector.max + bestGoalSector.min) / 2; // mid angle = average
        kickDistance = goalCenter.norm() + 10000.f; // approximation wtih margin
      }
    }

    void setGoalKickType()
    {
      // was:
      // bool left;
      // if (kickAngle > 15_deg) // kicking with right foot (toward our left)
      //   left = true;
      // else if (kickAngle < 15_deg) // kicking with left foot (toward our right)
      //   left = false;
      // else
      //   left = Random::bernoulli();
      // kickType = left? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickType = kickToLeft.state ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      kickAlignPrecisely = KickPrecision::precise;
    }
  };



  CRBEHAVIOUR(GotoBallAndKickSetPlayTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndKickSetPlayTask) {}

    void operator()()
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_BEGIN();

      kickToLeft.resetThresholds(params.kickFootThresholds);

      while (true)
      {
        CR_CHECKPOINT(proceed_to_kick);
        setSetPlayKickAngleAndDistance();
        setSetPlayKickType();
        gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);

        if (gotoBallAndKickTask.isSuccess())
          CR_EXIT_SUCCESS();
        else if (gotoBallAndKickTask.isFailure())
          CR_EXIT_FAILED();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoBallAndKickSetPlayTask,
    {,
      // (Rangef)(2000.f, 4000.f) forwardFastLongKickRange, ///< In this range the kick would reach the target
      // (Rangef)(1500.f, 2500.f) forwardFastKickRange, ///< In this range the kick would reach the target
      // (Rangef)(1000.f, 2000.f) walkForwardsLongKickRange, ///< In this range the kick would reach the target
      // (Rangef)(0.f, 1500.f) walkForwardsKickRange, ///< In this range the kick would reach the target
      (Rangef)(2000.f, 5000.f) forwardFastLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(1600.f, 2500.f) forwardFastKickRange,      ///< In this range the kick would reach the target
      (Rangef)(800.f, 2000.f) walkForwardsLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange,        ///< In this range the kick would reach the target
      (float)(750.f)  exclusionRadius,             ///< Radius around the ball for a legal kick in
      (float)(100.f)  exclusionThreshold,          ///< A 10cm tolerance for the exclusion radius
      (float)(1000.f) upfieldObstacleFreeShotDist, ///< The distance we shoot at if we find a sector with no obstacles (1m)
      (float)(2000.f) maxKickInDistanceNonGoalie,  ///< Maximum distance we are allowing the non goalie robots to shoot at in a kickIn
      (float)(5000.f) goalKickInLongShot,          ///< Distance the goalie shoots upfield if it finds an obstacle free path
      (Angle)(10_deg) minKickSectorSize,
      (float)(500.f) duelObstacleDistance,
      (float)(400.f) duelBallDistance,
      (Vector2f)(100.f, 100.f) ballKickableDistance, ///< when the ball is closer than this to front/side of the feet, we assume we were about to kick
      (Vector2f)(150.f, 150.f) ballNotKickableDistance, ///< when the ball is closer than this to front/side of the feet, we assume we were about to kick
      // (Angle)(15_deg) switchKickThreshold,  // hysteresis value for switching which foot we kick with when the kickAngle changes
      (HystThresholdsf)(-70, 70) kickFootThresholds,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    // READS(REGameInfo);
    READS(GameState);
    READS(REBehaviourParams);
    READS(FrameInfo);
    READS(RobotDimensions);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    KickPrecision kickAlignPrecisely;
    // was: bool kickToLeft = false;
    Comparatorf kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ true };

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    ObstacleSkills obstacleSkills {env};

    GotoBallAndKickTask gotoBallAndKickTask {env};
    
    /// @brief Sets the angles and distance for the different set plays for an indirect kick
    void setSetPlayKickAngleAndDistance() 
    {
      //Needs to be moved out of here
      // Form the obstacle sectors around the ball
      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
      
      // From the obstacle sectors, detect the best teammate sector and distance
      (void)kickSectors;
      std::tuple <Rangea, float, bool> bestPassSector = obstacleSkills.getBestPassSector(kickSectors);
      std::tuple <Rangea, bool> bestFreeSector = obstacleSkills.getBestFreeSector(kickSectors, getFreeSectorRange());

      // For the goal kick in, we try to shoot the ball upfield long as a priority
      if (theGameState.isGoalKick())
      { // Try shooting upfield 5m
        if (std::get<1>(bestFreeSector) && isRelativeKickInsideField(std::get<0>(bestFreeSector), params.goalKickInLongShot))
        {
          ACTGRAPH_FMT("GOAL_KICK: long shot with free sectors");
          kickDistance = params.goalKickInLongShot;
          kickAngle = (std::get<0>(bestFreeSector).max + std::get<0>(bestFreeSector).min) / 2;
        } // If not possible, try passing to a teammate
        // else if (std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector), std::get<1>(bestPassSector)))
        // {
        //   kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
        //   kickDistance = std::get<1>(bestPassSector);
        // } // If not possible, just shoot upfield 85 cm
        else
        { 
          ACTGRAPH_FMT("GOAL_KICK: default shot");
          kickDistance = params.goalKickInLongShot;
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(kickDistance)).angle();
        }
      } // For the other kick-ins, we first try to pass as a priority. Then attempt to shoot upfield with obstacle avoidance. If none work, just shoot upfield
      else if (theGameState.isFreeKick())
      { // Try passing to a teammate
        // if (std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector), std::get<1>(bestPassSector)))
        // {
        //   kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
        //   kickDistance = (std::get<1>(bestPassSector) > params.maxKickInDistanceNonGoalie) ? params.maxKickInDistanceNonGoalie : std::get<1>(bestPassSector);
        // } // If not possible, try shooting upfield with obstacle avoidance
        // else 
        if (std::get<1>(bestFreeSector) && isRelativeKickInsideField(std::get<0>(bestFreeSector), params.upfieldObstacleFreeShotDist))
        {
          ACTGRAPH_FMT("KI/FK/CK: shot with sectors");
          kickDistance = params.upfieldObstacleFreeShotDist;
          kickAngle = (std::get<0>(bestFreeSector).max + std::get<0>(bestFreeSector).min) / 2;
        } // If not possible, just shoot upfield
        else 
        {
          ACTGRAPH_FMT("KI/FK/CK: default shot");
          kickDistance = params.maxKickInDistanceNonGoalie;
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(kickDistance)).angle();
        }
      } 

      // Used for debugging
      ACTGRAPH_FMT("kickAngle = {}", kickAngle);
      ACTGRAPH_FMT("kickDistance = {}", kickDistance);
    }

    void setSetPlayKickType()
    {
      // was:
      // if (!kickToLeft && kickAngle > (0_deg + params.switchKickThreshold)) // kicking with right foot (toward our left)
      //   kickToLeft = true;
      // else if (kickToLeft && kickAngle < (0_deg - params.switchKickThreshold)) // kicking with left foot (toward our right)
      //   kickToLeft = false;
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickAlignPrecisely = KickPrecision::notPrecise;

      kickType = kickToLeft.state ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
    }

    bool isRelativeKickInsideField(Rangea angleR, float distance) {
        return theFieldDimensions.isInsideField(
          theRobotPose.toFieldCoordinates(
            Vector2f(
              (cosf(angleR.max + angleR.min) / 2) * distance,
              (sinf(angleR.max + angleR.min) / 2) * distance
          )));
    }

    /// @brief  Gets the shot's default target location on the field that the robot shoots for.
    ///         Used when there are no free sectors or when no teammates are found.
    /// @param  setPlay the current set play 
    /// @return Returns the target position on the field for the ball.
    Vector2f getDefaultShotLocation(float shotDistance) {
      // Side Kick-in: 85cm and _|_ to the side line
      if (theGameState.isThrowIn()) {
        return Vector2f(
            theFieldBall.recentBallEndPositionOnField().x(), 
            theFieldDimensions.yPosLeftTouchline - shotDistance *
                  ((sign(theFieldBall.recentBallEndPositionOnField().y())) ? (1) : (-1))
        ); 
      } // Corner Kick-in: 85cm and 45_deg from the corner
      else if (theGameState.isCornerKick()) 
      {
        return Vector2f(
            theFieldDimensions.xPosOpponentGoalLine - 
                  (shotDistance / sqrt(2)), 
            (theFieldDimensions.yPosLeftTouchline - (shotDistance / sqrt(2))) *
                  ((sign(theFieldBall.recentBallEndPositionOnField().y())) ? (1) : (-1))
        );
      } // Goal Kick-in: 85cm and upfield from the ball's location
      else if (theGameState.isGoalKick())
      {
        return Vector2f(
            theFieldBall.recentBallEndPositionOnField().x() +
                  shotDistance,
            theFieldBall.recentBallEndPositionOnField().y()
        );
      } // Pushing free kick
      else if (theGameState.isDirectFreeKick()) {
                // If we can shoot upfield, shoot upfield
                return (theFieldDimensions.isInsideField(
                            Vector2f(theFieldBall.recentBallEndPositionOnField().x() + shotDistance,
                                     theFieldBall.recentBallEndPositionOnField().y()))
                            ? Vector2f(theFieldBall.recentBallEndPositionOnField().x() + shotDistance,
                                       theFieldBall.recentBallEndPositionOnField().y())
                            :
                            // If we can't shoot left/right based on if we are on the right/left side of the field
                            Vector2f(theFieldBall.recentBallEndPositionOnField().x(),
                                     (theFieldBall.recentBallEndPositionOnField().y() > 0.0f
                                          ? theFieldBall.recentBallEndPositionOnField().y() - shotDistance
                                          : theFieldBall.recentBallEndPositionOnField().y() + shotDistance)));
                // TODO indirect free kick
      } // Default case
      else 
        return Vector2f(0.0f, 0.0f);
    }

    /// @brief Forms a limit range within which the kick must happen
    /// @return Returns the angle range for the kick (Field relative)
    Rangea getFreeSectorRange() 
    {
      if (theGameState.isGoalKick()) 
        return Rangea(-30_deg, 30_deg);
      else if (theGameState.isThrowIn())
        return theFieldBall.recentBallEndPositionOnField().y() > 0 ? Rangea(-180_deg, 0_deg) : Rangea(0_deg, 180_deg);
      else if (theGameState.isCornerKick())
        return theFieldBall.recentBallEndPositionOnField().y() > 0 ? Rangea(-180_deg, -90_deg) : Rangea(180_deg, 90_deg);
      else
        return Rangea(-180_deg, 180_deg);
    }

    // FIXME - change to use sgn function instead
    //
    /// @brief Used to quickly check the sign of a float variable
    /// @param x The float variable to check the sign of
    /// @return True of positive, False for negative
    bool sign(float x) {
      return (x >= 0.0) ? (true) : (false);
    }
  };
  


  //Isolate Passing Mechanism into it's own task
  //Do we dribble kick or pass task
  CRBEHAVIOUR(PassTask)
  {
    CRBEHAVIOUR_INIT(PassTask){}

    void operator()(void)
    {
        CRBEHAVIOUR_BEGIN();
        //Whilst pass viable and away from Goal, Pass!,!!! Make sure not in duel !!!
        CR_CHECKPOINT(Pass_Ball);
        while (true)
        {
          //Need Headmotion whilst passing


          choosePassTarget();
          passField=theRobotPose.toFieldCoordinates(passTargetOnField);
          ACTGRAPH_FMT("passTargetOnField: {}", passField);
          //draw passTargetOnFieldview
          drawCircle();
          //gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely,500.f);

          gotoBallAndPassTask(passTargetOnField); //kickSectors already in RoboRelative co-ords
        
          if (gotoBallAndPassTask.isSuccess())
            CR_EXIT_SUCCESS();
          else if ((theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f)) || (getCheckpointDuration()>params.timePassTargetExpire) || duelling || !passAvailable)
            CR_EXIT_FAILED();
          else
            CR_YIELD();
        }
    }
    private:
    DEFINES_PARAMS(PassTask,
    {,
      (float)(1000.f)dribbleDistance,
      (Angle)(10_deg) minKickSectorSize,
      (float)(15000.f)timePassTargetExpire,

    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(FieldBall);

    //Params
    //Angle kickAngle;
    //float kickDistance;
    //KickInfo::KickType kickType;
    //KickPrecision kickAlignPrecisely;

    Vector2f passTargetOnField;
    //Best Pass sector to determine whether to pass and then to have target
    std::tuple <Rangea, float, bool> bestPassSector;

    bool passAvailable=true;//If enter into this task then passAvailable true
    //bool opponentInWay=false;
    //bool havePassed=false; //Needs Implementation
    bool duelling=false; //Needs Implementation
    Vector2f passField;
    //How do we determine if we have properly executed a pass though (and not lost possession?)

    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
    ObstacleSkills obstacleSkills {env};

    GotoBallAndKickTask gotoBallAndKickTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};

    GotoBallAndPassTask gotoBallAndPassTask{env};
    GotoBallAndShootTask gotoBallAndShootTask{env};


    void choosePassTarget()
    {
      //Needs to be moved out of here
      // Form the obstacle sectors around the ball
      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
      (void)kickSectors;
      bestPassSector = obstacleSkills.getBestPassSector(kickSectors);

      //Is pass target good enough, do we get a pass sector? Is target within field boundaries
      // if(std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector),std::get<1>(bestPassSector)))
      // { 
      //   passTargetOnField=passSectorToPassTarget(bestPassSector);//Convert to field co-ords kicktarget
      //   passAvailable=true;
      // }
      // else
      //   passAvailable=false;

      //If we decide to move to using gotoBallAndKickTask
      //kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
      //kickDistance = std::get<1>(bestPassSector);
      // If not possible, try shooting upfield with obstacle avoidance
      if(std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector),std::get<1>(bestPassSector)))
      { 
        //kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
        //kickDistance = std::get<1>(bestPassSector);
        //Get estimate where we are kicking too
        passTargetOnField=passSectorToPassTarget(bestPassSector);//Convert to field co-ords kicktarget
        //Basic fill in to get kick direction
        //setGoalKickType();
        passAvailable=true;
      }
      else
      {
        passAvailable=false;
        (void)passTargetOnField;
      }
    }

    Vector2f passSectorToPassTarget(std::tuple <Rangea, float, bool> passSector)
    {
      Vector2f passTarget= Vector2f(
      (cosf((std::get<0>(passSector).max + std::get<0>(passSector).min)/2)) * (std::get<1>(passSector)),
      (sinf((std::get<0>(passSector).max + std::get<0>(passSector).min)/2)) * (std::get<1>(passSector))
      );
      return(passTarget);
    }

    //Copied Code from above to ensure pass stays within field boundaries
    bool isRelativeKickInsideField(Rangea angleR, float distance) 
    {
        return theFieldDimensions.isInsideField(
          theRobotPose.toFieldCoordinates(
            Vector2f(
              (cosf(angleR.max + angleR.min) / 2) * distance,
              (sinf(angleR.max + angleR.min) / 2) * distance
          )));
    }
    
    void drawCircle()
    {
      const float radius = 50.f;
      CIRCLE("behaviour:GotoBallSkills:passTargetOnField", passField.x(),passField.y(), radius, 500, Drawings::solidPen,
              ColorRGBA::blue, Drawings::noPen,ColorRGBA::blue);
    }


  };



  //Do we dribble kick or pass task
  CRBEHAVIOUR(DribbleKickOrPassTask)
  {
    CRBEHAVIOUR_INIT(DribbleKickOrPassTask) {}

    void operator()(void)
    {
      kickToLeft.resetThresholds(params.kickFootThresholds);

      CRBEHAVIOUR_LOOP()
      {//opponentInWay not used yet
        while((!passAvailable || opponentInWay) &&
        !(theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f)))
        {//Whilst no pass viable and away from Goal, move upfield!
          CR_CHECKPOINT(Move_Upfield);
          // choose initial pass target and don't change once decided
          //If works will transfer out of Move_Upfield
          choosePassTarget();
          //checkOpponentPos();
          //Whilst no teammates have been detected move forward with the ball 

          setKickDownfieldParams();
          gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely, 500.f);

          //When DRIBBLING IMPLEMENTED
          //gotoBallAndKickBestOptionTask();
          ACTGRAPH_FMT("NO PASS TARGET!!");
          CR_YIELD();
        }
        //Whilst pass viable and away from Goal, Pass!,!!! Make sure not in duel !!!
        CR_CHECKPOINT(Pass_Ball);
        //while (!gotoBallAndPassTask.isSuccess() &&
        //!(theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f)) && (getCheckpointDuration()<params.timePassTargetExpire) && !duelling && passAvailable)
        while(!passTask.isSuccess() && !passTask.isFailure())
        {//If 
          
          //Need Headmotion whilst passing
          //Not passing anywhere relevant need debug view to see why


          //AC- Try using KickSectors now
          //shotIsViable();
          //passTargetOnField = passSectorToPassTarget(obstacleSkills.getBestPassSector(obstacleSkills.populateKickSectorsAtBall()));
          //commonSkills.recordPassTarget(passTargetPlayer, passTargetOnField);
          //gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          //gotoBallAndPassTask(theRobotPose.toRobotCoordinates(Vector2f(0.f,0.f)));
          //choosePassTarget();
          //passField=theRobotPose.toFieldCoordinates(passTargetOnField);
          //addActivationGraphOutput(fmt::format("passTargetOnField: {} {}",passField.x(),passField.y()));

          //drawCircle();
          //gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely,500.f);
          //My updated chooseFirstPassTarget returns robot relative co-ords already
          //gotoBallAndPassTask(theRobotPose.toRobotCoordinates(passTargetOnField));
          //gotoBallAndPassTask(passTargetOnField); //Are kickSectors not already in RoboRelative co-ords
          passTask();
          CR_YIELD();
        }
        //Is resetting pass Available here best option?
        //Don;t need if as if failed needs to be redone?
        //passAvailable=false;
        //CR_YIELD();
        //Only shoot if we have succeeded in passing
        while(!theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField) && 
        (theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentGoalArea - 500.f))&& havePassed)
        {//If we are close to the goal, SHOOT!
          CR_CHECKPOINT(Shoot_Goal);
          gotoBallAndShootTask();
          CR_YIELD();
        }
        CR_YIELD();
      }
    }
    private:
    DEFINES_PARAMS(DribbleKickOrPassTask,
    {,
      (float)(1000.f)dribbleDistance,
      (Angle)(10_deg) minKickSectorSize,
      (float)(15000.f)timePassTargetExpire,
      (HystThresholdsf)(-70, 70) kickFootThresholds,
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(FieldBall);

    //Params
    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    KickPrecision kickAlignPrecisely;
    Comparatorf kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ true };
    Vector2f passTargetOnField;
    //Best Pass sector to determine whether to pass and then to have target
    std::tuple <Rangea, float, bool> bestPassSector;

    bool passAvailable=false;
    bool opponentInWay=false;
    bool havePassed=false; //Needs Implementation
    //bool duelling=false; //Needs Implementation
    //Vector2f passField;
    //How do we determine if we have properly executed a pass though (and not lost possession?)
    //Functions imported

    CommonSkills commonSkills {env};
    HeadSkills headSkills{env};
    ObstacleSkills obstacleSkills {env};

    GotoBallAndKickTask gotoBallAndKickTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
    PassTask passTask {env};

    GotoBallAndPassTask gotoBallAndPassTask{env};
    GotoBallAndShootTask gotoBallAndShootTask{env};
    
    //Functions
    void choosePassTarget()
    {
      //Needs to be moved out of here
      // Form the obstacle sectors around the ball
      const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
      (void)kickSectors;
      bestPassSector = obstacleSkills.getBestPassSector(kickSectors);

      //Is pass target good enough, do we get a pass sector? Is target within field boundaries
      // if(std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector),std::get<1>(bestPassSector)))
      // { 
      //   passTargetOnField=passSectorToPassTarget(bestPassSector);//Convert to field co-ords kicktarget
      //   passAvailable=true;
      // }
      // else
      //   passAvailable=false;

      //If we decide to move to using gotoBallAndKickTask
      //kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
      //kickDistance = std::get<1>(bestPassSector);
      // If not possible, try shooting upfield with obstacle avoidance
      if(std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector),std::get<1>(bestPassSector)))
      { 
        kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
        kickDistance = std::get<1>(bestPassSector);
        //Get estimate where we are kicking too
        passTargetOnField=passSectorToPassTarget(bestPassSector);//Convert to field co-ords kicktarget
        //Basic fill in to get kick direction
        setGoalKickType();
        passAvailable=true;
      }
      else
      {
        passAvailable=false;
        (void)passTargetOnField;
      }
    }

    Vector2f passSectorToPassTarget(std::tuple <Rangea, float, bool> passSector)
    {
      Vector2f passTarget= Vector2f(
      (cosf((std::get<0>(passSector).max + std::get<0>(passSector).min)/2)) * (std::get<1>(passSector)),
      (sinf((std::get<0>(passSector).max + std::get<0>(passSector).min)/2)) * (std::get<1>(passSector))
      );
      return(passTarget);
    }

    //Copied Code from above to ensure pass stays within field boundaries
    bool isRelativeKickInsideField(Rangea angleR, float distance) 
    {
        return theFieldDimensions.isInsideField(
          theRobotPose.toFieldCoordinates(
            Vector2f(
              (cosf(angleR.max + angleR.min) / 2) * distance,
              (sinf(angleR.max + angleR.min) / 2) * distance
          )));
    }
    
    // void drawCircle()
    // {
    //   const float radius = 50.f;
    //   CIRCLE("behaviour:GotoBallSkills:passTargetOnField", passField.x(),passField.y(), radius, 500, Drawings::solidPen,
    //           ColorRGBA::blue, Drawings::noPen,ColorRGBA::blue);
    //   //ORIGIN("behaviour:GotoBallSkills:passTargetOnField",passTargetOnField.x(),passTargetOnField.y());
    //   // DEBUG_DRAWING("behaviour:GotoBallSkills:passTargetOnField", "drawingOnField")
    //   // {
    //   //   const float radius = 500.f;
    //   //   CIRCLE("behaviour:GotoBallSkills:passTargetOnField", theRobotPose.translation.x(), theRobotPose.translation.y(), radius, 500, Drawings::solidPen,
    //   //         ColorRGBA::blue, Drawings::noPen,ColorRGBA::blue);
    //   // }
    // }


     //AC -Kick Downfield, No passtarget available
    void setKickDownfieldParams()
    {
      setGoalKickAngleAndDistance();
      setGoalKickType();
    }

    //AC-Stolen from striker behaviour
        /// angle to goal in robot relative coordinates
    void setGoalKickAngleAndDistance()
    {
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGoalLine, 0.f};
      Vector2f goalCenter = theRobotPose.toRobotCoordinates(goalCenterOnField);

      kickAngle = goalCenter.angle();
      kickDistance = goalCenter.norm() + 1000.f; // approximation wtih margin

      if (goalCenter.norm() < 6000.f) // FIXME HACK
      {
        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBall();
        (void)kickSectors;
        Rangea bestGoalSector = obstacleSkills.getBestGoalSector(kickSectors);

        if (bestGoalSector.getSize() > params.minKickSectorSize)
        {
          kickAngle = (bestGoalSector.max + bestGoalSector.min) / 2; // mid angle = average
          kickDistance = goalCenter.norm() + 1000.f; // approximation wtih margin
        }
      }

      ACTGRAPH_FMT("kickAngle = {}", kickAngle);
      // addActivationGraphOutput(fmt::format("goalCenterAngle = {}", goalCenter.angle()));
    }

    void setGoalKickType()
    {
      // was:
      // bool left;
      // if (kickAngle > 15_deg) // kicking with right foot (toward our left)
      //   left = true;
      // else if (kickAngle < 15_deg) // kicking with left foot (toward our right)
      //   left = false;
      // else
      //   left = Random::bernoulli();
      // kickType = left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      updateKickFoot(kickToLeft, kickAngle, theFieldBall.positionRelative.y());

      kickAlignPrecisely = KickPrecision::notPrecise;

      kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
    }


  };





} // namespace v2025
} // namespace RE
