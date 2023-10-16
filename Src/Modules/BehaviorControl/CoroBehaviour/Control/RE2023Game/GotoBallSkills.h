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
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"


namespace CoroBehaviour
{
namespace RE2023
{

  // =====================================================================

  /**
   * Main entry point of the striker behaviour in playing state
   */
  CRBEHAVIOUR(GotoBallAndKickBestOptionTask)
  {
    CRBEHAVIOUR_INIT(GotoBallAndKickBestOptionTask) {}

    void operator()(bool indirect = false)
    {
      // we assume that a higher level behaviour will switch away from this
      // skill if the ball has not been seen for too long, so we just use
      // the current ball model as the real position and don't bother checking
      // when we saw it last
      CRBEHAVIOUR_LOOP()
      {
        // try not to walk into goal
        if (theFieldDimensions.isInsideOpponentGoal(theFieldBall.endPositionOnField))
        {
          commonSkills.standLookActive();
          CR_EXIT_SUCCESS();
        }

        updateKickParams(indirect);
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
    DEFINES_PARAMS(GotoBallAndKickBestOptionTask,
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
    });

    READS(RobotPose);
    READS(FieldDimensions);
    READS(ObstacleModel);
    READS(FieldBall);
    READS(GameInfo);
    READS(RobotInfo);

    Angle kickAngle;
    float kickDistance;
    KickInfo::KickType kickType;
    bool kickAlignPrecisely;

    CommonSkills commonSkills {env};
    ObstacleSkills obstacleSkills {env};
    GotoBallAndKickTask gotoBallAndKickTask {env};

    void updateKickParams(bool indirect)
    {
      if (indirect)
      {
        setIndirectKickAngleAndDistance();
        setIndirectKickType();
      }
      else if (isKickTowardsGoalBestOption())
      {
        setGoalKickAngleAndDistance();
        setGoalKickType();
      }
      addActivationGraphOutput(fmt::format("indirect = {}", indirect));
    }

    /// @brief Sets the angles and distance for the different set plays for an indirect kick
    void setIndirectKickAngleAndDistance() 
    {
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
          kickDistance = params.goalKickInLongShot;
          kickAngle = (std::get<0>(bestFreeSector).max + std::get<0>(bestFreeSector).min) / 2;
        } // If not possible, try passing to a teammate
        else if (std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector), std::get<1>(bestPassSector)))
        {
          kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
          kickDistance = std::get<1>(bestPassSector);
        } // If not possible, just shoot upfield 85 cm
        else
        { 
          kickDistance = params.exclusionRadius + params.exclusionThreshold;
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(theGameInfo.setPlay, kickDistance)).angle();
        }
      } // For the other kick-ins, we first try to pass as a priority. Then attempt to shoot upfield with obstacle avoidance. If none work, just shoot upfield
      else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK || theGameInfo.setPlay == SET_PLAY_KICK_IN || theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK)
      { // Try passing to a teammate
        if (std::get<2>(bestPassSector) && isRelativeKickInsideField(std::get<0>(bestPassSector), std::get<1>(bestPassSector)))
        {
          kickAngle = (std::get<0>(bestPassSector).max + std::get<0>(bestPassSector).min) / 2;
          kickDistance = (std::get<1>(bestPassSector) > params.maxKickInDistanceNonGoalie) ? params.maxKickInDistanceNonGoalie : std::get<1>(bestPassSector);
        } // If not possible, try shooting upfield with obstacle avoidance
        else if (std::get<1>(bestFreeSector) && isRelativeKickInsideField(std::get<0>(bestFreeSector), params.upfieldObstacleFreeShotDist))
        {
          kickDistance = params.upfieldObstacleFreeShotDist;
          kickAngle = (std::get<0>(bestFreeSector).max + std::get<0>(bestFreeSector).min) / 2;
        } // If not possible, just shoot upfield
        else 
        {
          kickDistance = params.exclusionRadius + params.exclusionThreshold;
          kickAngle = theRobotPose.toRobotCoordinates(getDefaultShotLocation(theGameInfo.setPlay, kickDistance)).angle();
        }
      } 

      // Used for debugging
      addActivationGraphOutput(fmt::format("kickAngle = {}_deg", toDegrees(kickAngle)));
      addActivationGraphOutput(fmt::format("kickDistance = {}", kickDistance));
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

    bool isKickTowardsGoalBestOption() const
    {
      // TODO: needs a real implementation
      return true;
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

      addActivationGraphOutput(fmt::format("kickAngle = {}_deg", toDegrees(kickAngle)));
      // addActivationGraphOutput(fmt::format("goalCenterAngle = {}_deg", toDegrees(goalCenter.angle())));
    }

    void setIndirectKickType()
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
      if (kickDistance < 3000.f)
        kickType = left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      else
        kickType = left ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
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
      if (obstacleSkills.isOpponentDangerNearBall())
        kickType = left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
      else if (params.forwardFastLongKickRange.isInside(kickDistance))
      {
        kickAlignPrecisely = true;
        kickType = left ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      }
      else if (params.forwardFastKickRange.isInside(kickDistance))
        kickType = left ? KickInfo::forwardFastLeft : KickInfo::forwardFastRight;
      else if (params.walkForwardsLongKickRange.isInside(kickDistance))
        kickType = left ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
      else if (theRobotInfo.isGoalkeeper()) // clearance kick
        kickType = left ? KickInfo::walkForwardsLeftLong : KickInfo::walkForwardsRightLong;
        // kickType = left ? KickInfo::forwardFastLeftLong : KickInfo::forwardFastRightLong;
      else
        kickType = left ? KickInfo::walkForwardsLeft : KickInfo::walkForwardsRight;
    }

    /// @brief Used to quickly check the sign of a float variable
    /// @param x The float variable to check the sign of
    /// @return True of positive, False for negative
    bool sign(float x) {
      return (x >= 0.0) ? (true) : (false);
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
      addActivationGraphOutput(fmt::format("kickAngle = {}_deg, kickDistance = {}", toDegrees(kickAngle), kickDistance));
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

} // RE2023
} // CoroBehaviour
