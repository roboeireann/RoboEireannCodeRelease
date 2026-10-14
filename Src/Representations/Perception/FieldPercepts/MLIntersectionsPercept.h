#pragma once

#include "Math/Eigen.h"
#include "Streaming/AutoStreamable.h"
#include <vector>

/**
 * @struct MLIntersectionsPercept
 * Field line intersections detected by the ML multi-class detector.
 * Unlike IntersectionsPercept, entries carry no line-index references because
 * they originate from bounding boxes rather than fitted scan-line segments.
 */
STREAMABLE(MLIntersectionsPercept,
{
  STREAMABLE(Intersection,
  {
    ENUM(IntersectionType,
    {,
      L,
      T,
      X,
    });
    Intersection() = default;
    Intersection(IntersectionType t, const Vector2f& p, const Matrix2f& c)
    {
      type = t; pos = p; cov = c;
    },

    (IntersectionType) type,
    (Vector2f) pos, /**< Field coordinates of the intersection, relative to the robot */
    (Matrix2f) cov, /**< Covariance of pos */
  });

  void draw() const,

  (std::vector<Intersection>) intersections,
});

inline void MLIntersectionsPercept::draw() const {}
