/**
 * Top level handling of team coordination
 *
 * @author Rudi Villing
 */

#include "REHybridBehaviourModule.h"


CORO_DEFINE((BehaviourContainer, noTeamCoordinationTask))
{
  CORO_BEGIN()

  while (true)
  {
    // teamBehaviourStatusSkills.recordTimeToReachBall(TimeToReachBall());
    // teamBehaviourStatusSkills.recordTeammateRoles(TeammateRoles());
    // teamBehaviourStatusSkills.recordPlayerRole(PlayerRole());

    CORO_YIELD();  
  }

  CORO_END()
}
