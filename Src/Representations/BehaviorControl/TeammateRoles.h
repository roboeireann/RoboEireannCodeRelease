/**
 * @file TeammateRoles.h
 *
 * This file declares a representation of a team role assignment in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(TeammateRoles,
{
  // adds elements to the roles vector behind the scenes if needed
  int operator[](const size_t i) const;
  int& operator[](const size_t i);
  
  /***** separate header from streamable members, note comma at end *****/,

  (std::vector<int>) roles, /**< The role assignment for all robots in the team (which is its index in the player set, i.e. how many active robots are behind it). */
  (int)(-1) captain, /**< The number of the robot which calculated this role assignment. -1 means none yet */
  (unsigned)(0) timestamp, /**< The timestamp when this role assignment has been calculated. */
});
