/**
 * @file TeammateRoles.cpp
 *
 * This file implements a representation of a team role assignment in the 2019 behavior.
 *
 * @author Arne Hasselbring
 */

#include "TeammateRoles.h"

#include "Representations/BehaviorControl/PlayerRole.h"


int& TeammateRoles::operator[](const size_t i)
{
  while(roles.size() <= i)
    // roles.push_back(-1);
    roles.push_back(static_cast<int>(PlayerRole::none));
  return roles[i];
}

int TeammateRoles::operator[](const size_t i) const
{
  if(roles.size() <= i)
    // return -1;
    return static_cast<int>(PlayerRole::none);
  return roles[i];
}
