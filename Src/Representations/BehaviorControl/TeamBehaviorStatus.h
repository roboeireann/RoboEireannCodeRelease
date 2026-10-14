/**
 * @file TeamBehaviorStatus.h
 *
 * This file declares a representation of the team behavior status in the 2019 behavior.
 *
 * @author Arne Hasselbring
 * @author Rudi Villing (modifications)
 * 
 * @note [BH2021]: Ported from B-Human 2021 Code Release
 */

#pragma once

#include "Representations/BehaviorControl/PlayerRole.h"
#include "Representations/BehaviorControl/TeammateRoles.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/BehaviorControl/REPassStatus.h"
#include "Tools/Communication/BHumanMessageParticle.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

/**
 * @struct RETeamPlayStatus
 * representation of teamplay such as passes, touches by teammates, etc.
 */
STREAMABLE(RETeamPlayStatus,
{
  /******** streamable members follow (note comma at end of comment) ********/,

  // (int)(-1) activePassKicker, ///< player who intends to pass (-1 for none)
  // (int)(-1) activePassReceiver, ///< player who should receive the intended pass (-1 for none)
  // // (int)(-1) prevPassReceiver, ///< the player number of the previous pass target (-1 for none)
  // (int)(0) numPasses, ///< substantial pass attempts (ball travelled substantially)
  (int)(0) numKickingRobotsSinceLastSetPlay, ///< a kicking robot executes one or more kicks, increment each time robot changes
});


STREAMABLE(TeamBehaviorStatus, COMMA public BHumanCompressedMessageParticle<TeamBehaviorStatus>
{
//   ENUM(TeamActivity,
//   {,
//     noTeam,
// 
//     // RE additions
//     minimalTeam,
//     dynamicBallHandlingTeam,
//   });
//   static constexpr TeamActivity numOfTeamActivities = numOfTeamActivitys;

  void draw() const;
  
    /**** streamable fields follow - note comma at end of this line ****/,

  // (TeamActivity)(noTeam) teamActivity,
  (TimeToReachBall) timeToReachBall,
  (TeammateRoles) teammateRoles,
  (PlayerRole) role,
  (REPassStatus) passStatus, // deprecated
  (RETeamPlayStatus) teamPlayStatus,
});

// typedef TeamBehaviorStatus TeamBehaviourStatus; // alias with "correct" spelling
