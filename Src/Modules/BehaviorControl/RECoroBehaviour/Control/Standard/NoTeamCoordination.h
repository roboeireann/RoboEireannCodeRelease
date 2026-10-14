/**
 * @file NoTeamCoordination.h
 *
 * This task implements the default team coordination behaviour (i.e. no
 * coordination). It is adapted from the BH2021 NoTeam system card.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/TeamBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

namespace RE
{

  CRBEHAVIOUR(NoTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(NoTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // teamBehaviourStatusSkills.recordTeamActivity(TeamBehaviorStatus::noTeam);
        teamBehaviourStatusSkills.recordTimeToReachBall(TimeToReachBall());
        teamBehaviourStatusSkills.recordTeammateRoles(TeammateRoles());
        teamBehaviourStatusSkills.recordPlayerRole(PlayerRole());
        
        CR_YIELD();
      }
    }

  private:
    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};
  };

} // namespace RE