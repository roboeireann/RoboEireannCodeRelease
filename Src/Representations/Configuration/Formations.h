/**
 * @file Formations.h
 * 
 * This file defines the information associated with a formation
 * and the list of all known formations.
 * 
 * The formations selected for different situations are configured in 
 * Tactics.cfg and will usually be a subset of the full set of
 * formations listed in Formations::formations. Formations are identified
 * and referenced by their formationId.
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Representations/BehaviorControl/PlayerRole.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Range.h"

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <string>
#include <vector>
#include <unordered_map>


// Note: in the following all positions/poses refer to field coordinates
// unless otherwise specified. Therefore we won't tag "OnField" suffix to
// every variable name

STREAMABLE(Formations,
{
  ENUM(FormationId,
  {,
    // 5v5
    kickoffForward121,
    defendingKickoff121,
    defending121,
    attacking121,

    // 7v7
    kickoffForward231,
    defendingKickoff231,
    defending231,
    attacking231,

    attacking222,
  });

  // an attempt at reasonable position names that can be used in various 
  // formations up to 11 v 11
  ENUM(FormationRole,
  {,
    noFormationRoleSpecified,
    goalie,
    centreBack, // if there is only one central defender, or possibly taking on the role of sweeper
    centreBackR,
    centreBackL,
    fullBackR, // wider defenders or wing backs
    fullBackL, // wider defenders or wing backs
    centreMid, // if there is only one, can be CDM or CAM also
    centreMidR,
    centreMidL,
    midR, // wider midfield players/wingers
    midL,
    forward, // if there is only one
    forwardR,
    forwardL,
  });

  STREAMABLE(Formation,
  {,
    (FormationId) formationId, // this is somewhat redundant, but including it makes it more likely that we'll 
                               // detect a mismatching enum and formation in the config file

    // the following are the positions in priority order (i.e. the first 2 positions listed
    // will be the positions occupied if there are only 2 players on the field). Typically,
    // but not always, the first position will be the goalie.
    (std::vector<FormationRole>) roles, // the formation role associated with the formation position at the same index
    (std::vector<Vector2f>) positions, 
    
    // the formation can shift according to the ball position - the following params specify the limits
    (float) yShift, // the amount the formation center can be shifted left
    (float) yScale, // the scaling that will be applied in the most shifted position (usually y compression to make formation narrower)
    (float) xAnchor, // The anchor for shifts. Points in the formation are scaled relative to their distance from xAnchor.
    (Rangef) xShift, // the min (farthest back) and max (farthest forward) the formation center can be x-shifted 
                   // relative to the default postions above (when shift=zero)
    (Rangef) xScale, // the scaling that will be applied in min and max positions (scaling is 1.0 at zero shift)
  });
  // TODO - how to relate above to ball position, particularly for x

  /***** separate header from streamable members, note comma at end *****/,

  (ENUM_INDEXED_ARRAY(Formation, FormationId)) formations,
});
