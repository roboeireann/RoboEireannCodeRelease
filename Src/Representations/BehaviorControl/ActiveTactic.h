/**
 * @file ActiveTactic.h
 *
 * Information about the active tactic etc.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Configuration/Formations.h"
#include "Representations/BehaviorControl/PlayerRole.h"

#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"



/** limited information for sharing with teammates in team comms */
STREAMABLE(ActiveTacticStatus, 
{

  /******** streamable members follow (note comma at end of comment) ********/,

  (Formations::FormationId) formationId,
  (Formations::FormationRole) formationRole,
  (PlayerRole::Type) situationRole,
});


/** more complete information that ActiveTacticStatus required for behaviour to execute */
STREAMABLE_WITH_BASE(ActiveTactic, ActiveTacticStatus, 
{

  /******** streamable members follow (note comma at end of comment) ********/,

  // members related to this robot's role in the active tactic

  (Vector2f) assumedBallPositionOnField, // where the ball is or is assumed to be, further constrained by the Tactics config (used to shift formation)
  (bool) attacking, // do we have the ball and/or is the ball far in the attacking half?
  (Pose2f) formationPose, // the current formation pose for this player (on the field), modified by ball effect

  // members related to the whole team (self + teammates) in the active tactic
  // This is needed by the TeamLocator and possibly other modules

  (std::vector<Pose2f>) formationPoses, // the current formation poses of the team (on the field), modified by ball effect
  (std::vector<Formations::FormationRole>) formationRoles,
  (std::vector<PlayerRole::Type>) situationRoles, // dynamic roles based on the situation  
});
