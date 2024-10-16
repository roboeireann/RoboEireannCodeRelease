/**
 * @file TeamPlayersObstacleModel.h
 *
 * Declaration of a representation that represents the combined obstacle models from the team.
 *
 * @author Katharina Gillmann
 */

#pragma once

#include <vector>
#include "Tools/Modeling/Obstacle.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/ObstacleModel.h"

/**
 * @struct TeamPlayersObstacleModel
 * a combined obstacle model.
 */
STREAMABLE(TeamPlayersObstacleModel,
{
  void verify() const;
  void draw() const,

  (std::vector<Obstacle, Eigen::aligned_allocator<Obstacle>>) obstacles, /**< 'field pose' (field coordinates) and covariance of combined obstacles */
});
