/**
 * @file TeammatesLocationModel.h
 *
 * Representation of best known teammate positions based on communicated
 * info and anything we know about their typical behaviour (normally just
 * based on the expected tactic pose)
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Modeling/ObstacleModel.h"

#include <vector>

/**
 * @struct TeammatesLocationModel
 * best guess as to teammate locations
 */
STREAMABLE(TeammatesLocationModel,
{
  ENUM(LocationType,
  {,
    formation, // following the formation positions
    independent, // independent of the formation positions, e.g. ball player going to the ball
    penalized, // as it says, the robot is penalized and there is no useful location info available
  });

  STREAMABLE(Location,
  {,
    (int)(-1) playerNumber,
    (LocationType)(formation) locationType,
    (unsigned)(0) timeWhenLocationCommunicated,
    (Pose2f) pose, ///< estimated field pose of player
  });

  void draw() const;
  
  /**** separate header from streamable fields, note comma at end of this line ****/,

  (std::vector<Location>) locations, 
});
