/**
 * this file implements the units (functions and classes) that will be tested 
 * in our Catch2 tutorial.
 *
 * This makes it a bit more like how Catch2 would really be used
 * rather than the simple tutorial online
 */

#include "tutorial_src.h"

#include <cmath>



unsigned int factorial( unsigned int number ) 
{
    // FIXME: this implementation is intentionally has one small bug.
    // When testing uncovers it, comment out this next line and uncomment the
    // line below instead to get the correct solution

    // return (number <= 1) ? number : factorial(number-1)*number;
    
    return (number <= 1) ? 1 : factorial(number-1)*number;
}


// ############################################################################
// ############################################################################
// ############################################################################


float normalizeDegrees(float deg) 
{
  // NOTE: this implementation has an intentional bug for some values of deg
  if (-180.0f <= deg && deg < 180.0)
    return deg;
  else
  {
    deg = std::fmod(deg, 360.f); // the remainder after dividing by 360

    // FIXME - the following 4 lines of code only work for positive angles.
    // Some negative angles give erroneous results. When the tests find this
    // comment out the following 4 lines and uncomment the lines beneath instead

    // if (deg >= 180.0f)
    //   return deg - 360.0f;
    // else
    //   return deg;

    if (deg >= 180.0f)
      return deg - 360.0f;
    else if (deg < -180.0f)
      return deg + 360.0f;
    else
      return deg;
  }
}


// ############################################################################
// ############################################################################
// ############################################################################

void RobotHelper::setStriker(bool val)
{
  striker = val;
  needsUpdate = true;
}

void RobotHelper::setBallDistance(float val)
{
  ballDistance = val;
  needsUpdate = true;
}

void RobotHelper::setTargetDistance(float val)
{
  targetDistance = val;
  needsUpdate = true;
}

float RobotHelper::getWalkSpeed() 
{ 
  updateWalkAndKick(); 
  return walkSpeed; 
}

float RobotHelper::getKickStrength() 
{ 
  updateWalkAndKick(); 
  return kickStrength; 
}

void RobotHelper::updateWalkAndKick()
{
  if (!needsUpdate)
    return;

  needsUpdate = false; // we're doing the update now so mark it done

  // FIXME - the following code is incomplete and has bugs.
  // It doesn't consider whether this robot is the striker or not, but should.
  // It also doesn't implement any linear interpolation.

  if (ballDistance > 150.0f)
    walkSpeed = 1.0f;
  else
    walkSpeed = 0.0f;
  
  if (targetDistance > 3000.0f)
    kickStrength = 1.0f;
  else 
    kickStrength = 0.0f;
}



// ############################################################################
// ############################################################################
// ############################################################################

// TODO: self test - this one is incomplete. You can try to finish it off by making
// sure the tests pass. (Actually you would need to add more tests also.) Refer
// to tutorial_src.h to get the "specification" of what this function should
// do.

void Game::setState(State newState)
{
  switch (state)
  {
  default:
  case IDLE:
    if ((newState == READY) || (newState == PLAYING))
      state = newState;
    break;

  case READY:
    // TODO
    break;

  case SET:
    // TODO
    break;

  case PLAYING:
    // TODO
    break;
  }
}
