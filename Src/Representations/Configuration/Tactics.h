/**
 * @file Tactics.h
 *
 * Information about the tactics configuration for this game.
 * This configuration includes the formations to use for different situations
 * 
 * TODO: attack vs defence aggression and when to switch between them etc.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Configuration/Formations.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(Tactics,
{
  STREAMABLE(Formation,
  {
    STREAMABLE(BallEffect,
    {,
      (float) xAnchor, // scaled x-position relative to anchor will determine effect on formation
      (Rangef) xRange, // ball positions will be clamped to this range (which must include the xAnchor) for effect calc
      (Vector2f) scale, // ball positions (relative to anchor) will be scaled by this amount for effect calc
    });
    
    /***** separate header from streamable members, note comma at end *****/,

    (std::vector<Formations::FormationId>) formationIds, // the formations, ordered by number of active players high to low
    (BallEffect) ballEffect,
  });

  /***** separate header from streamable members, note comma at end *****/,

  (int) maxPlayers, // the max number of active players these tactics are designed for
  (Formation) ourKickoffFormation,
  (Formation) opponentKickoffFormation,
  (Formation) attackingFormation,
  (Formation) defendingFormation,  
});
