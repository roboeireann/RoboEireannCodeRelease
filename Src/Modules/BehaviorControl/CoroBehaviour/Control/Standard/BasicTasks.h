/**
 * @file BasicTasks.h
 *
 * A collection of simple tasks that just stand and either look forward
 * or look around.
 * 
 * These may useful during behaviour development as placeholders
 * or for behaviours where the ready, set, or return from penalized behaviours
 * don't need to do anything much.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/CoroBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"



namespace CoroBehaviour
{

  /**
   * Just stand in place and look around (without any special attention on the ball)
   */
  CRBEHAVIOUR(StandLookActiveTask)
  {
    CRBEHAVIOUR_INIT(StandLookActiveTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
        commonSkills.stand(/* high */ true);

        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
  };


  /**
   * Just stand in place and look forwards
   */
  CRBEHAVIOUR(StandLookForwardTask)
  {
    CRBEHAVIOUR_INIT(StandLookForwardTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        commonSkills.standLookForward(/* high */ true);
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills {env};
  };


  /**
   * Placeholder task that just stands and looks forward.
   * 
   * Unlike the StandLookForwardTask, it is possible to give this task
   * a different name - e.g. the name of the real behaviour that will replace
   * it since it is usually used as a placeholder during development
   * for a behaviour to be added later
  */
  CRBEHAVIOUR(PlaceholderTask)
  {
    PlaceholderTask(BehaviourEnv& env, const char * name="PlaceholderTask") : CoroBehaviour(env, name) {}
    // CRBEHAVIOUR_INIT(PlaceholderTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.activityStatus(BehaviorStatus::rePlaceholder);
        commonSkills.standLookForward(/* high */ true);
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills  {env};
  };



  /**
   * Placeholder task that stands and looks forward and exits with SUCCESS
   * after a short delay
   * 
   * Like the PlaceholderTask you can give it a name other than the default name
  */
  CRBEHAVIOUR(PlaceholderSuccessTask)
  {
    PlaceholderSuccessTask(BehaviourEnv& env, const char * name="PlaceholderSuccessTask") : CoroBehaviour(env, name) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.activityStatus(BehaviorStatus::rePlaceholder);
        commonSkills.standLookForward(/* high */ true);
        if (getCoroDuration() > SUCCESS_DURATION_MS)
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    const unsigned SUCCESS_DURATION_MS = 500;

    CommonSkills commonSkills  {env};
  };

} // CoroBehaviour