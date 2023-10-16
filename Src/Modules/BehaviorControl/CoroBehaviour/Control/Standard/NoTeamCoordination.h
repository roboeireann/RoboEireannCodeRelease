/**
 * @file NoTeamCoordination.h
 *
 * This task implements the default team coordination behaviour (i.e. no
 * coordination). It is adapted from the BH2021 NoTeam system card.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/TeamBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

namespace CoroBehaviour
{

  CRBEHAVIOUR(NoTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(NoTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        teamBehaviourStatusSkills.teamActivity(TeamBehaviorStatus::noTeam);
        teamBehaviourStatusSkills.timeToReachBall(TimeToReachBall());
        teamBehaviourStatusSkills.teammateRoles(TeammateRoles());
        teamBehaviourStatusSkills.playerRole(PlayerRole());
        
        CR_YIELD();
      }
    }

  private:
    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};
  };

} // CoroBehaviour2022