/**
 * @file TeamBehaviorStatus2024.h
 *
 * the shared status of team coordination/behaviour for 2024
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Configuration/Formations.h"

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(TeamBehaviourStatus2024,
{
  // STREAMABLE(Role,
  // {,
  // });

  /******** streamable members follow (note comma at end of comment) ********/,

  (Formations::FormationId) formationId,
  // (TimeToReachBall) timeToReachBall,
  // (TeammateRoles) teammateRoles,
  // (Role) role,
});
