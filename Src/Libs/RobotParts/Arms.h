#pragma once

#include "Streaming/Enum.h"

namespace Arms
{
  ENUM(Arm,
  {,
    left,
    right,
  });

  // anyArm and bothArms are always used in different contexts.
  // we overload numOfArms for this to maintain compatibility with existing BH code
  constexpr Arm anyArm = numOfArms;
  constexpr Arm bothArms = numOfArms;
}
