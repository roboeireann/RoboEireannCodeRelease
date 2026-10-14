#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"

/**
 * @struct MLCirclePercept
 * Centre circle detected by the ML multi-class detector.
 * Unlike CirclePercept, the position is derived from the bounding box centre
 * and should not be used for precise localisation — it is most useful as a
 * "seen" flag and rough field-side indicator.
 */
STREAMABLE(MLCirclePercept,
{
  void draw() const,

  (Vector2f)(Vector2f::Zero()) pos,
  (Matrix2f)((Matrix2f() << 1.f, 0.f, 0.f, 1.f).finished()) cov,
  (bool)(false) wasSeen,
});

inline void MLCirclePercept::draw() const {}
