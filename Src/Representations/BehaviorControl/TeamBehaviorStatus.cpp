/**
 * @file TeamBehaviorStatus.cpp
 *
 * Some function implementations for TeamBehaviorStatus.
 * This representation is largely deprecated from 2024 on
 *
 * @author Rudi Villing
 */

#include "TeamBehaviorStatus.h"

#include "Tools/Debugging/DebugDrawings3D.h"

void TeamBehaviorStatus::draw() const
{
  DEBUG_DRAWING3D("representation:TeamBehaviorStatus:role", "robot")
  {
    if (role.isBallPlayer())
      SPHERE3D("representation:TeamBehaviorStatus:role", 0, 0, 700, 30, ColorRGBA::red.darker(0.5f));
  }
}