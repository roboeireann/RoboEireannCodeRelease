/**
 * @file FieldCoverage.h
 *
 * Declaration to send information about the field coverage.
 *
 * @author Nicole Schrader
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(FieldCoverage,
{
  STREAMABLE(GridLine,
  {,
    (int) y,
    (std::vector<unsigned>)() timestamps,
  }),

  (std::array<GridLine, 12>) lines,
});
