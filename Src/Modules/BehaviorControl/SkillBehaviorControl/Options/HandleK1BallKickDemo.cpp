/**
 * @file HandleK1BallKickDemo.cpp
 *
 * This file implements the HandleK1BallKickDemo option for kick demos.
 * 
 * @author James Florin Petri
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) HandleK1BallKickDemo)
{
  initial_state(soccer)
  {
    action
    {
      DemoSoccerK1();
    }
  }
}
