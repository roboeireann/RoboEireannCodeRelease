/**
 * @file DemoSoccerK1.cpp
 *
 * This file implements the DemoSoccerK1 skill.

 * It is a variant of the DemoSccer focused for the K1. The robot searches for
 * the ball (rotates on the spot if not in front of it) and kicks it once seen.
 * See DemoGoToBallAndKickK1 skill for the kick.
 * 
 * @author James Florin Petri
 */

#include "SkillBehaviorControl.h"

option((SkillBehaviorControl) DemoSoccerK1)
{
  initial_state(searching)
  {
    transition
    {
      // Only play the ball if we saw it some time before far away or very recently.
      if((theFieldBall.ballWasSeen(4000) && theBallModel.estimate.position.squaredNorm() > sqr(500.f)) ||
         theFieldBall.ballWasSeen(2000))
        goto playing;
    }
    action
    {
      DemoSearchForBall();
    }
  }

  state(playing)
  {
    transition
    {
      // Do not play the ball if we have not seen it for a while or we have lost it while it was close.
      if(!theFieldBall.ballWasSeen(4000) ||
         (!theFieldBall.ballWasSeen(2000) && theBallModel.estimate.position.squaredNorm() <= sqr(500.f)))
        goto searching;
    }
    action
    {
      DemoGoToBallAndKickK1();
    }
  }
}
