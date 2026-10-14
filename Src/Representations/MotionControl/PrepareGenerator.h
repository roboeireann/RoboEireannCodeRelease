/**
 * @file PrepareGenerator.h
 *
 * This is a generator (called by the motion engine) to generate motion
 * phases for the Prepare motion. (See PrepareEngine.h/cpp for details)
 *
 * @author Rudi Villing
 */

#pragma once

#include "Streaming/Function.h"
#include "Tools/Motion/MotionGenerator.h"
#include "Streaming/AutoStreamable.h"

STREAMABLE_WITH_BASE(PrepareGenerator, MotionGenerator,
{
  BASE_HAS_FUNCTION,
});
