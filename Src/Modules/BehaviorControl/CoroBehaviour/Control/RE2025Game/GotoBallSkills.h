/**
 * @file GotoSkills.h
 *
 * This file contains some RE2019 based goto skills.
 * It has been adapted from the RoboEireann 2019 behaviour
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 * @author James Petri
 * @author Andy Lee Mitchell
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"

#include "BallAndKickSkills.h"

#include "Tools/TextLogging.h"


namespace CoroBehaviour
{
namespace RE2025
{

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
          //           theFieldBall.endPositionOnField.x() > theFieldDimensions.xPosHalfWayLine)
          else if ((theTeamData.countActiveTeammatesKickedSinceRestart() == 0) &&
                   passing.update(theFieldBall.endPositionOnField.x()))
          //  (theFieldBall.endPositionOnField.x() >
          //   (wasPassing ? theFieldDimensions.xPosHalfWayLine - 250.f : theFieldDimensions.xPosHalfWayLine - 100)))
          {
            CR_CHECKPOINT(proceed_to_pass);
            setPassAngleAndDistance();
            setPassKickType();
            gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely);
            // passingNow = true;
          }
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
      (Angle)(10_deg) minKickSectorSize,
      (float)(500.f) duelObstacleDistance,
      (float)(400.f) duelBallDistance,
      (Vector2f)(100.f, 100.f) ballKickableDistance, ///< when the ball is closer than this to front/side of the feet, we assume we can kick
      (Vector2f)(150.f, 150.f) ballNotKickableDistance, ///< when the ball is farther than this from front/side of the feet, we assume we can no longer kick
      (float)(75.f) duellingBallMovedDistance, ///< when the ball moves by this amount in a duel, we assume an opponent moved it
      (HystThresholdsf)(-70, 70) kickFootThresholds,
      (Vector2f)(1600.f, 1500.f) kickAwayMax, ///< we'll go to the ball and kick it away at these distances from the ground line and y=0 line
      (unsigned)(3000) kickAwayBallSeenTimeoutMs, ///< we'll allow the ball to be obstructed briefly on the way to kicking it away
      (HystThresholdsf)(-250.f, -100.f) passXThresholds, // we'll pass if x > highThresh or not if x < low threshold
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameInfo);
    READS(BehaviourParams);
    READS(FrameInfo);
    READS(RobotDimensions);
    READS(TeamData);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;
    Vector2f duellingObstacleCenter;
    unsigned timeWhenDuelDelayStarted = 0;
    bool ballWasKickable = false;
    Vector2f prevEndPositionOnField; // 0,0
    Comparatorf passing { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ true };
    Comparatorf kickToLeft { /* stateWhenInputLow */ false, /* stateWhenInputHigh */ true, /* initialStateLow */ false };

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};



    bool checkDuellingDelayRequired()
    {
      ACTGRAPH_FMT("ballWasKickable: {},\nballPos: {}", ballWasKickable, theFieldBall.positionRelative);

      if (theBehaviourParams.demoDuellingDelayMs == 0)
        return false;

      // are we already in a delay
      if (timeWhenDuelDelayStarted &&
          (theFrameInfo.getTimeSince(timeWhenDuelDelayStarted) < theBehaviourParams.demoDuellingDelayMs))
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
        if(o.type != Obstacle::Type::goalpost) // && o.type != Obstacle::Type::unknown)
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
        ACTGRAPH_FMT("kickTowardsGoal angle={:.1f}", toDegrees(kickAngle));
      }
      else
      {
        setDuelKickAngle();
        setDuelKickType();
        ACTGRAPH_FMT("duelling angle={:1.f}", toDegrees(kickAngle));
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
      if (theGameInfo.isGoalkeeper())
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
      Vector2f leftHalfwayLine {theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline};
      Vector2f rightHalfwayLine {theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosRightSideline};
      Vector2f centreHalfwayLine {theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosCenterGoal};
      Rangea bestKickSector;
      bool left = theFieldBall.endPositionOnField.y() > 0.f ? true : false;
      if (theGameInfo.isGoalkeeper())
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
      ACTGRAPH_FMT("kickAngleDeg: {} deg", kickAngle.toDegrees());
      if (kickAngle < -45_deg) // Right foot aiming left
      {
        kickToLeft.state = true;
      }
      else if (kickAngle > 45_deg) // Left foot aiming right
      {
        kickToLeft.state = false;
      }
      else
      {
        kickToLeft.update(theFieldBall.positionRelative.y());
      }

      kickAlignPrecisely = false;

      kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
    }


    /// angle to goal in robot relative coordinates
    void setGoalKickAngleAndDistance()
    {
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGroundLine, 0.f};
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
      // kickToLeft.update(kickAngle);
      ACTGRAPH_FMT("kickAngleDeg: {} deg", kickAngle.toDegrees());
      if (kickAngle < -45_deg) // Right foot aiming left
      {
        kickToLeft.state = true;
      }
      else if (kickAngle > 45_deg) // Left foot aiming right
      {
        kickToLeft.state = false;
      }
      else
      {
        kickToLeft.update(theFieldBall.positionRelative.y());
      }
      
      kickAlignPrecisely = false;

      bool canTakeLongShot = (theRobotPose.translation.x() >= theBehaviourParams.longShotAtGoalMinX);

      // how strong to make the kick?
      if (obstacleSkills.isOpponentDangerNearBallDuel(500.f))
      {
        // if (theRobotPose.translation.x() > theFieldDimensions.centerCircleRadius)
        if (canTakeLongShot && (theBehaviourParams.walkForwardsLongKickAtGoalRange.getSize() > 0))
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        else
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      }
      else if (canTakeLongShot && theBehaviourParams.forwardFastLongKickAtGoalRange.isInside(kickDistance))
      {
        kickAlignPrecisely = true;
        kickType = kickToLeft.state ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      }
      else if (canTakeLongShot && theBehaviourParams.forwardFastKickAtGoalRange.isInside(kickDistance))
        kickType = kickToLeft.state ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else if (canTakeLongShot && theBehaviourParams.walkForwardsLongKickAtGoalRange.isInside(kickDistance))
        kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      else if (theGameInfo.isGoalkeeper()) // clearance kick
        kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      else
      {
        // one of the walk kicks must be enabled
        if (theBehaviourParams.walkForwardsKickAtGoalRange.getSize() > 0)
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
        else if (theBehaviourParams.walkForwardsLongKickAtGoalRange.getSize() > 0)
          kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        else
          TLOGF_ABORT(TextLogging::get("GotoBallAndKickBestOption"),
                     "At least one walk kick must have a valid range in behaviourParams.cfg");
      }

      ACTGRAPH_FMT("kickToLeft: {}\nkickType: {}\nkickAlignPrecisely: {}", kickToLeft.state,
                      TypeRegistry::getEnumName(kickType), kickAlignPrecisely);
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
      // addActivationGraphOutput(fmt::format("nearestOpponent distance = {}mm, angle = {}_deg", nearestOpponent.center.norm(), toDegrees(nearestOpponent.center.angle())));
      // addActivationGraphOutput(fmt::format("angle on field = {}_deg", toDegrees(theRobotPose.toFieldCoordinates(Pose2f(kickAngle)).rotation)));
    }

    void setDuelKickType()
    {
      ACTGRAPH_FMT("kickAngleDeg: {} deg", kickAngle.toDegrees());
      if (kickAngle < -45_deg) // Right foot aiming left
      {
        kickToLeft.state = true;
      }
      else if (kickAngle > 45_deg) // Left foot aiming right
      {
        kickToLeft.state = false;
      }
      else
      {
        kickToLeft.update(theFieldBall.positionRelative.y());
      }
      // we want the kick to be as fast as possible
      kickAlignPrecisely = false;
      
      kickType = kickToLeft.state ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      // kickType = KickInfo::walkSidewardsLeftFootToLeft;
    }

    void setPassAngleAndDistance()
    {
      Vector2f leftPassingLimit {theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosLeftSideline};
      Vector2f rightPassingLimit {theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosRightSideline};

      // we want to kick into free area towards the goal (or either side if in the middle of the field)
      Vector2f leftKickLimit;
      Vector2f rightKickLimit;
      if ((std::abs(theFieldBall.endPositionOnField.y()) > theFieldDimensions.centerCircleRadius))
      {
        bool left = theFieldBall.endPositionOnField.y() > 0.f ? true : false;
        if (left)
        {
          leftKickLimit = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoalArea);
          rightKickLimit = Vector2f(theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosRightSideline);
        }
        else
        {
          leftKickLimit = Vector2f(theFieldBall.endPositionOnField.x() + 200.f, theFieldDimensions.yPosLeftSideline);
          rightKickLimit = Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea);
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
        Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGroundLine, 0.f};
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
        Vector2f leftPassingLimit {theFieldBall.endPositionOnField.x() + (left*200.f), (left*theFieldDimensions.yPosRightSideline)};
        Vector2f rightPassingLimit {theFieldBall.endPositionOnField.x() - (left*200.f), (left*theFieldDimensions.yPosRightSideline)};

        Rangea bestKickSector;

        const std::list<SectorWheel::Sector>& kickSectors = obstacleSkills.populateKickSectorsAtBallToFieldArea(leftKickLimit, rightKickLimit);
        (void)kickSectors;
        bestKickSector = obstacleSkills.getBestFieldAreaSector(kickSectors);

        Vector2f targetRelative = theRobotPose.toRobotCoordinates(target);
        kickAngle = targetRelative.angle();
        kickDistance = 2*std::abs(target.y());
      }

      ACTGRAPH_FMT("PASS: kickDistance: {}", kickDistance);
      ACTGRAPH_FMT("PASS: kickAngle = {}", kickAngle);
    }

    void setPassKickType()
    {
      ACTGRAPH_FMT("kickAngleDeg: {} deg", kickAngle.toDegrees());
      if (kickAngle < -45_deg) // Right foot aiming left
      {
        kickToLeft.state = true;
      }
      else if (kickAngle > 45_deg) // Left foot aiming right
      {
        kickToLeft.state = false;
      }
      else
      {
        kickToLeft.update(theFieldBall.positionRelative.y());
      }
      kickAlignPrecisely = false;

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
      Angle testKickAngleOnField = toDegrees(tempFieldPose.rotation);
      // addActivationGraphOutput(fmt::format("robot-relative angle = {}, field-relative angle = {}", toDegrees(testKickAngle), testKickAngleOnField));
      return (std::abs(testKickAngleOnField) < limit);
    }

    bool isKickAngleLeft(Angle testKickAngle)
    {
      Pose2f tempFieldPose =  theRobotPose.toFieldCoordinates(Pose2f(testKickAngle, 0.f, 0.f));
      Angle testKickAngleOnField = toDegrees(tempFieldPose.rotation);
      // addActivationGraphOutput(fmt::format("robot-relative angle = {}, field-relative angle = {}", toDegrees(testKickAngle), testKickAngleOnField));
      return (testKickAngleOnField > 0_deg);
    }
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
      (Rangef)(2000.f, 5000.f) forwardFastLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(1600.f, 2500.f) forwardFastKickRange,      ///< In this range the kick would reach the target
      (Rangef)(800.f, 2000.f) walkForwardsLongKickRange,  ///< In this range the kick would reach the target
      (Rangef)(0.f, 1000.f) walkForwardsKickRange,        ///< In this range the kick would reach the target
      (float)(750.f)  exclusionRadius,             ///< Radius around the ball for a legal kick in
      (float)(100.f)  exclusionThreshold,          ///< A 10cm tolerance for the exclusion radius
      (float)(1000.f) upfieldObstacleFreeShotDist, ///< The distance we shoot at if we find a sector with no obstacles (1m)
      (float)(2000.f) maxKickInDistanceNonGoalie,  ///< Maximum distance we are allowing the non goalie robots to shoot at in a kickIn
      (float)(5000.f) goalKickInLongShot,          ///< Distance the goalie shoots upfield if it finds an obstacle free path
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameInfo);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;

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
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGroundLine, 0.f};
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
      bool left;

      if (kickAngle > 15_deg) // kicking with right foot (toward our left)
        left = true;
      else if (kickAngle < 15_deg) // kicking with left foot (toward our right)
        left = false;
      else
        left = Random::bernoulli();
      
      kickType = left? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      kickAlignPrecisely = true;
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
      (Angle)(15_deg) switchKickThreshold,  // hysteresis value for switching which foot we kick with when the kickAngle changes
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameInfo);
    READS(BehaviourParams);
    READS(FrameInfo);
    READS(RobotDimensions);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;
    bool kickToLeft = false;

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
      std::tuple <Rangea, bool> bestFreeSector = obstacleSkills.getBestFreeSector(kickSectors, getFreeSectorRange(theGameInfo.setPlay));

      // For the goal kick in, we try to shoot the ball upfield long as a priority
      if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK)
      { // Try shooting upfield 5m
        if (std::get<1>(bestFreeSector) && isRelativeKickInsideField(std::get<0>(bestFreeSector), params.goalKickInLongShot))
        {
          ACTGRAPH_FMT("GOAL_KICK: long shot with free sectors");
          kickDistance = params.goalKickInLongShot;
          kickAngle = (std::get<0>(bestFreeSector).max + std::get<0>(bestFreeSector).min) / 2;
        } 
        else
        { 
          ACTGRAPH_FMT("GOAL_KICK: default shot");
          kickDistance = params.goalKickInLongShot;
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(theGameInfo.setPlay, kickDistance)).angle();
        }
      } // For the other kick-ins, we first try to pass as a priority. Then attempt to shoot upfield with obstacle avoidance. If none work, just shoot upfield
      else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK)
      { // Try passing to a teammate
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
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(theGameInfo.setPlay, kickDistance)).angle();
        }
      } 

      // Used for debugging
      ACTGRAPH_FMT("kickAngle = {}", kickAngle);
      ACTGRAPH_FMT("kickDistance = {}", kickDistance);
    }

    void setSetPlayKickType()
    {
      // has the kickAngle changed enough that we should switch which foot we kick with?
      if (!kickToLeft && kickAngle > (0_deg + params.switchKickThreshold)) // kicking with right foot (toward our left)
        kickToLeft = true;
      else if (kickToLeft && kickAngle < (0_deg - params.switchKickThreshold)) // kicking with left foot (toward our right)
        kickToLeft = false;

      kickAlignPrecisely = false;

      // how strong to make the kick?
      kickType = kickToLeft ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
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
    Vector2f getDefaultShotLocation(uint8_t setPlay, float shotDistance) {
      // Side Kick-in: 85cm and _|_ to the side line
      if (setPlay == SET_PLAY_KICK_IN) {
        return Vector2f(
            theFieldBall.recentBallEndPositionOnField().x(), 
            theFieldDimensions.yPosLeftSideline - shotDistance *
                  ((sign(theFieldBall.recentBallEndPositionOnField().y())) ? (1) : (-1))
        ); 
      } // Corner Kick-in: 85cm and 45_deg from the corner
      else if (setPlay == SET_PLAY_CORNER_KICK) 
      {
        return Vector2f(
            theFieldDimensions.xPosOpponentGroundLine - 
                  (shotDistance / sqrt(2)), 
            (theFieldDimensions.yPosLeftSideline - (shotDistance / sqrt(2))) *
                  ((sign(theFieldBall.recentBallEndPositionOnField().y())) ? (1) : (-1))
        );
      } // Goal Kick-in: 85cm and upfield from the ball's location
      else if (setPlay == SET_PLAY_GOAL_KICK)
      {
        return Vector2f(
            theFieldBall.recentBallEndPositionOnField().x() +
                  shotDistance,
            theFieldBall.recentBallEndPositionOnField().y()
        );
      } // Pushing free kick
      else if (setPlay == SET_PLAY_PUSHING_FREE_KICK) {
                // If we can shoot upfield, shoot upfield
        return (theFieldDimensions.isInsideField(Vector2f(
                              theFieldBall.recentBallEndPositionOnField().x() + shotDistance,
                              theFieldBall.recentBallEndPositionOnField().y())) ?
                  Vector2f(theFieldBall.recentBallEndPositionOnField().x() + shotDistance,
                           theFieldBall.recentBallEndPositionOnField().y()) :
                // If we can't shoot left/right based on if we are on the right/left side of the field
                  Vector2f(theFieldBall.recentBallEndPositionOnField().x(),
                          (theFieldBall.recentBallEndPositionOnField().y() > 0.0f ?
                              theFieldBall.recentBallEndPositionOnField().y() - shotDistance :
                              theFieldBall.recentBallEndPositionOnField().y() + shotDistance))
        );
      } // Default case
      else 
        return Vector2f(0.0f, 0.0f);
    }

    /// @brief Forms a limit range within which the kick must happen
    /// @param setPlay < The current set play
    /// @return Returns the angle range for the kick (Field relative)
    Rangea getFreeSectorRange(uint8_t setPlay) 
    {
      if (setPlay == SET_PLAY_GOAL_KICK) 
        return Rangea(-30_deg, 30_deg);
      else if (setPlay == SET_PLAY_KICK_IN)
        return theFieldBall.recentBallEndPositionOnField().y() > 0 ?
            Rangea(-180_deg, 0_deg) : Rangea(0_deg, 180_deg);
      else if (setPlay == SET_PLAY_CORNER_KICK)
        return theFieldBall.recentBallEndPositionOnField().y() > 0 ?
            Rangea(-180_deg, -90_deg) : Rangea(180_deg, 90_deg);
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

    Vector2f passTargetOnField;
    //Best Pass sector to determine whether to pass and then to have target
    std::tuple <Rangea, float, bool> bestPassSector;

    bool passAvailable=true;//If enter into this task then passAvailable true
    bool duelling=false; //Needs Implementation
    Vector2f passField;

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

      // If not possible, try shooting upfield with obstacle avoidance
      if(std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector),std::get<1>(bestPassSector)))
      { 
        //Get estimate where we are kicking too
        passTargetOnField=passSectorToPassTarget(bestPassSector);//Convert to field co-ords kicktarget
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
          gotoBallAndKickTask(kickAngle, kickType, kickAlignPrecisely,500.f);

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
        {          
          passTask();
          CR_YIELD();
        }

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

    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(FieldBall);

    //Params
    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;
    Vector2f passTargetOnField;
    //Best Pass sector to determine whether to pass and then to have target
    std::tuple <Rangea, float, bool> bestPassSector;

    bool passAvailable=false;
    bool opponentInWay=false;
    bool havePassed=false; //Needs Implementation

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
      Vector2f goalCenterOnField {theFieldDimensions.xPosOpponentGroundLine, 0.f};
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
      // addActivationGraphOutput(fmt::format("goalCenterAngle = {}_deg", toDegrees(goalCenter.angle())));
    }

    void setGoalKickType()
    {
      bool left;

      if (kickAngle > 15_deg) // kicking with right foot (toward our left)
        left = true;
      else if (kickAngle < 15_deg) // kicking with left foot (toward our right)
        left = false;
      else
        left = Random::bernoulli();

      kickAlignPrecisely = false;

      // how strong to make the kick?
      kickType = left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
    }


  };


} // RE2025
} // CoroBehaviour
