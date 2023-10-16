/**
 * @file Threads/Cognition2D.h
 *
 * This file declares the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 * @author Arne Hasselbring
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Framework/FrameExecutionUnit.h"

/**
 * @class Cognition
 *
 * The execution unit for the cognition thread.
 */
class Cognition2D : public FrameExecutionUnit
{
public:

  Cognition2D();
  bool beforeFrame() override;
  void beforeModules() override;
  void afterModules() override;
};
