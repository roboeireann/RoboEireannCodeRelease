/**
 * @file BehaviourFormations.h
 * 
 * The possible team formations that can be used
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Representations/BehaviorControl/PlayerRole.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Pose2f.h"
#include <string>
#include <unordered_map>

// Note: in the following all positions/poses refer to field coordinates
// unless otherwise specified. Therefore we won't tag "OnField" suffix to
// every variable name

STREAMABLE(BehaviourFormations,
{
  // ENUM(PoseMapping,
  // {,
  //   byPlayerNumber, // one pose per player in player number order starting from player 1
  //   byPosition, // poses to be selected based on which player is closest
  //   byRole, // select based on behaviour role
  // });

  STREAMABLE(Formation,
  {,
    // (PoseMapping) poseMapping,
    (std::vector<Pose2f>) poses,
  });

  STREAMABLE(PositionConstraints,
  {,
    // (std::optional<PlayerRole::RoleType>)() selectByRole, // when selecting by nearest point
    // Note: optional specified as empty array in .cfg file if not present, or array with one element if present

    (Rangef) ballXRange,
    (Rangef) ballYRange,
    (Rangef) robotXRange, // the range of the robot itself
    (Rangef) robotYRange, // the range of the robot itself
  });

  STREAMABLE(TacticFormation,
  {,
    // TODO we need to do a better job of making it possible to flexibly
    // switch between quite different formations (e.g. for 5v5 vs 7v7)

    // when writing the config file, set positions for robot on the left by default.
    // These will be mirrored if necessary for a robot on the right
    (PositionConstraints) defenderConstraints,
    (PositionConstraints) winger1Constraints,
    (PositionConstraints) winger2Constraints,
  });

  // const Formation& getFormation(const std::string& name) const;
  // const TacticFormation& getTacticFormation(const std::string& name) const;

  /***** separate header from streamable members, note comma at end *****/,

  (std::optional<Formation>)() readyStateDefaultKickoffOurs,
  (std::optional<Formation>)() readyStateDefaultKickoffTheirs,

  (std::optional<TacticFormation>)() normalTacticFormation,

  // (std::vector<Formation>) formations, /* access by formation name via getFormation */
  // (std::vector<TacticFormation>) tacticFormations,
});
