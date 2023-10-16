/**
 * @file: ObstacleSkills.h
 * 
 * Skills related obstacle detection and avoidance etc
 * 
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/MotionSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BehaviorStatusSkills.h"

#include "Tools/BehaviorControl/SectorWheel.h"


namespace CoroBehaviour
{

  struct ObstacleSkills
  {
    ObstacleSkills(BehaviourEnv& env) : env(env) 
    {
      DECLARE_DEBUG_DRAWING("behaviour:ObstacleSkills:ballKickSectorWheel", "drawingOnField");
    }

    // ------------------------------------------------------------------------
    // stateless functions to call directly
    // ------------------------------------------------------------------------

    /**
     * simple wrapper for checking if a vector length is closer than some distance.
     * This makes most sense if the vector represents a position expressed
     * in robot-relative (or some other pose-relative) coordinates
     */
    static bool isCloserThan(const Vector2f& pos, float distance)
    {
      return pos.squaredNorm() < sqr(distance);
    }

    // if there are opponents near the ball we should usually act quickly
    // we ignore opponents walking away from the ball
    bool isOpponentDangerNearBall(float distanceThreshold = 1200.f)
    {
      for (auto obstacle : theObstacleModel.obstacles)
      {
        if ((obstacle.type == Obstacle::opponent) &&
            (isCloserThan(obstacle.center - theFieldBall.endPositionRelative, distanceThreshold)))
        {
          // TODO check that opponent is not walking away

          // if (obstacle.velocity.angle())
          //   ;

          return true;
        }
      }

      return false; // all clear
    }


    bool areTeammatesNearPointOnField(const Vector2f& pointOnField, float distanceThreshold = 1000.f)
    {
      for (auto obstacle : theObstacleModel.obstacles)
      {
        if (((obstacle.type == Obstacle::teammate) || (obstacle.type == Obstacle::fallenTeammate)) &&
            (isCloserThan(obstacle.center - pointOnField, distanceThreshold)))
        {
          return true;
        }
      }

      return false; // all clear
    }

    
    bool areOpponentsNearPointOnField(const Vector2f& pointOnField, float distanceThreshold = 1000.f)
    {
      for (auto obstacle : theObstacleModel.obstacles)
      {
        if (((obstacle.type == Obstacle::opponent) || (obstacle.type == Obstacle::fallenOpponent)) &&
            (isCloserThan(obstacle.center - pointOnField, distanceThreshold)))
        {
          return true;
        }
      }

      return false; // all clear
    }

    
    // populate sectors from a kick point (probably ball) in robot relative coords
    const std::list<SectorWheel::Sector>& populateKickSectors(const Vector2f& point, const Rangea& angleIgnore = Rangea(0,0))
    {
      sectorWheel.begin(point);

      // mark any no-go area
      if (angleIgnore.getSize() > 0)
        sectorWheel.addSector(angleIgnore, 1000.f, SectorWheel::Sector::erased);

      // mark the goal
      Vector2f goalLeft {theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal - theFieldDimensions.goalPostRadius};
      Vector2f goalRight {theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal + theFieldDimensions.goalPostRadius};
      // Vector2f goalCenter {theFieldDimensions.xPosOpponentGroundLine, 0.f};

      // sectorWheel.addSector(Rangea(theRobotPose.toRobotCoordinates(goalRight).angle(), theRobotPose.toRobotCoordinates(goalLeft).angle()), 
      //   theRobotPose.toRobotCoordinates(goalCenter).norm(), SectorWheel::Sector::goal);
      sectorWheel.addSector(theRobotPose.toRobotCoordinates(goalLeft), theRobotPose.toRobotCoordinates(goalRight),
                            SectorWheel::Sector::goal);

      // add obstacles from the obstacle model

      for (auto obstacle : theObstacleModel.obstacles)
      {
        SectorWheel::Sector::Type sectorType;

        switch (obstacle.type)
        {
          default:
            sectorType = SectorWheel::Sector::obstacle;
            break;

          case Obstacle::teammate:
          case Obstacle::fallenTeammate:
            sectorType = SectorWheel::Sector::teammate;
        }

        sectorWheel.addSector(obstacle.center, (obstacle.left - obstacle.right).norm()/2, sectorType);
      }

      return sectorWheel.finish();
    }

    const std::list<SectorWheel::Sector>& populateKickSectorsAtBall(const Rangea& angleIgnore = Rangea(-180_deg,-180_deg))
    {
      const std::list<SectorWheel::Sector>& sectors = populateKickSectors(theFieldBall.endPositionRelative, angleIgnore);

      // set origin to robot relative coordinates
      ORIGIN("behaviour:ObstacleSkills:ballKickSectorWheel", theRobotPose.translation.x(), theRobotPose.translation.y(), theRobotPose.rotation);
      DRAW_SECTOR_WHEEL("behaviour:ObstacleSkills:ballKickSectorWheel", sectors, theFieldBall.endPositionRelative);
      ORIGIN("behaviour:ObstacleSkills:ballKickSectorWheel", 0, 0, 0); // reset coordinates to field

      return sectors;
    }


    // goal sectors wider than minKickSectorSize that are not obstructed (if any)
    std::list<SectorWheel::Sector> getGoalSectors(const std::list<SectorWheel::Sector> &sectorList,
                                                  const Angle &minKickSectorSize = 10_deg)
    {
      std::list<SectorWheel::Sector> goalSectors;

      for (auto sector : sectorList)
      {
        if (sector.type == SectorWheel::Sector::goal && (sector.angleRange.getSize() > minKickSectorSize))
          goalSectors.push_back(sector);
      }

      return goalSectors;
    }

    // best goal sector (e.g. nearest and widest)
    Rangea getBestGoalSector(const std::list<SectorWheel::Sector> &sectorList)
    {
      Rangea angleRange(0,0);

      for (auto sector : sectorList)
      {
        if (sector.type == SectorWheel::Sector::goal && (sector.angleRange.getSize() > angleRange.getSize()))
          angleRange = sector.angleRange;
      }

      return angleRange;
    }

    // best passing sector (e.g. nearest and widest)
    std::tuple <Rangea, float, bool> getBestPassSector(const std::list<SectorWheel::Sector> &sectorList)
    {
      Rangea angleRange(0,0);
      float kickDistance = std::numeric_limits<float>::max();
      bool teammateFound = false;

      for (auto sector : sectorList)
      {
        // If the angle range is higher, the teammate is closer!
        if (sector.type == SectorWheel::Sector::teammate && (sector.angleRange.getSize() > angleRange.getSize())) {
          kickDistance = sector.distance;
          angleRange = sector.angleRange;
          teammateFound = true;
        }
      }

      return { angleRange, kickDistance, teammateFound };
    }
    
    // Finding the best free sector (no teammates or opponents)
    std::tuple <Rangea, bool> getBestFreeSector(const std::list<SectorWheel::Sector> &sectorList, const Rangea limits)
    {
      Rangea angleRange(0_deg, 0_deg);
      Angle deviation = abs(limits.max - limits.min);
      bool freeSectorFound = false;

      for (auto sector : sectorList)
      {
        // Checking if we can find a free sector and it is included into the sector limits. Also checking
        // if the current sector center is closer to the center of the limit sector (the deviation)
        if (sector.type == SectorWheel::Sector::free && abs(limits.getCenter() - sector.angleRange.getCenter()) < deviation
            && limits.isInside(theRobotPose.toFieldCoordinates(theRobotPose).rotation + sector.angleRange.getCenter())) 
        {
          angleRange = sector.angleRange;
          deviation = abs(limits.getCenter() - sector.angleRange.getCenter());
          freeSectorFound = true;
        }
      }

      return { angleRange, freeSectorFound };
    }

    // ======================================================================
    // team based obstacle info
    // ======================================================================

    float nearestOpponentDistance(int playerNumber)
    {
      float distance = std::numeric_limits<float>::max();

      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (teammate.number == playerNumber)
        {
          for (const Obstacle& obstacle : teammate.theObstacleModel.obstacles)
          {
            if (obstacle.type == Obstacle::opponent)
              distance = std::min(obstacle.center.norm(), distance);
          }
          break;
        }
      }
      return distance; // no opponent closer than distance to specified teammate
    }


    const Obstacle* nearestOpponentAttackingHalf(int playerNumber)
    {
      const Obstacle* closest = nullptr;
      float minDistance = std::numeric_limits<float>::max();

      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (teammate.number == playerNumber)
        {
          for (const Obstacle& obstacle : teammate.theObstacleModel.obstacles)
          {
            if (obstacle.type == Obstacle::opponent)
            {
              float distance = obstacle.center.norm();
              if ((distance < minDistance) && (theRobotPose.toFieldCoordinates(obstacle.center).x() > 0))
              {
                minDistance = distance;
                closest = &obstacle;
              }
            }
          }
          break;
        }
      }
      return closest;
    }



  private:
    BehaviourEnv& env;

    READS(RobotPose);
    READS(FieldBall);
    READS(ObstacleModel);
    READS(FieldDimensions);
    READS(TeamData);

    SectorWheel sectorWheel;
  };

} // CoroBehaviour
