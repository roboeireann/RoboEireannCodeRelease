/**
 * @file ActiveTactic.h
 *
 * Information about the active tactic etc.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Configuration/REFormations.h"
#include "Representations/BehaviorControl/PlayerRole.h"

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"



/** limited information for sharing with teammates in team comms */
STREAMABLE(REActiveTacticStatus, 
{

  /******** streamable members follow (note comma at end of comment) ********/,

  (REFormations::FormationId) formationId,
  (REFormations::FormationRole) formationRole,
  (PlayerRole::Type) situationRole,
});


/** more complete information that REActiveTacticStatus required for behaviour to execute */
STREAMABLE_WITH_BASE(REActiveTactic, REActiveTacticStatus, 
{
  void draw() const;

  /******** streamable members follow (note comma at end of comment) ********/,

  // members related to this robot's role in the active tactic

  (Vector2f) assumedBallPositionOnField, // where the ball is or is assumed to be, further constrained by the Tactics config (used to shift formation)
  (bool) attacking, // do we have the ball and/or is the ball far in the attacking half?
  (Pose2f) formationPose, // the current formation pose for this player (on the field), modified by ball effect

  // members related to the whole team (self + teammates) in the active tactic
  // This is needed by the TeamLocator and possibly other modules

  (std::vector<Pose2f>) formationPoses, // the current formation poses of the team (on the field), modified by ball effect
  (std::vector<REFormations::FormationRole>) formationRoles,
  (std::vector<PlayerRole::Type>) situationRoles, // dynamic roles based on the situation  
});
