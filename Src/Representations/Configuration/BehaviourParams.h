/**
 * @file BehaviourParams.h
 * 
 * Common parameters for Behaviour to simplify loading from one place and
 * to provide some loadable parameters separate from parameters defined
 * in code that are not loadable
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(BehaviourParams,
{,
  // demo related params
  (int)(0) demoDuellingDelayMs, ///< a delay in taking actions while duelling, only applied to the weaker team for demos, 0 is disabled
});
