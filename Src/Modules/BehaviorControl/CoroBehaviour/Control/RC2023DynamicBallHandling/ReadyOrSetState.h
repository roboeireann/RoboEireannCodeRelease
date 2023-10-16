/**
 * @file ReadyOrSetStateTask.h
 *
 * This task implements a look around behaviour for Ready and Set state
 * in the dynamic ball handling challenge
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"


namespace CoroBehaviour
{
namespace RC2023
{

  CRBEHAVIOUR(ReadyOrSetStateTask)
  {
    CRBEHAVIOUR_INIT(ReadyOrSetStateTask) {}

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

} // RC2023

} // CoroBehaviour2023