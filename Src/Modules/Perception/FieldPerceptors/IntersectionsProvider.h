/**
 * @file IntersectionsProvider.h
 *
 * Geometric-only intersection classifier. Replaces IntersectionsCandidatesProvider +
 * IntersectionsClassifier with a single module that has no neural network dependency.
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Perception/FieldPercepts/IntersectionsPercept.h"
#include "Representations/Perception/FieldPercepts/LinesPercept.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Perception/MeasurementCovariance.h"

MODULE(IntersectionsProvider,
{,
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(FieldDimensions),
  REQUIRES(LinesPercept),
  REQUIRES(MeasurementCovariance),
  USES(RobotPose),
  PROVIDES(IntersectionsPercept),
  LOADS_PARAMETERS(
  {,
    (float) maxAllowedAngleDifference, /**< Max deviation from 90° (radians). */
    (float) maxIntersectionGap,        /**< Max distance (mm) from line endpoint to intersection to classify as ending there. */
  }),
});

class IntersectionsProvider : public IntersectionsProviderBase
{
  void update(IntersectionsPercept& intersectionsPercept) override;
  void enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const;
  bool isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const;
  template<typename T>
  float getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b,
                       const Eigen::Matrix<T, 2, 1>& target,
                       Eigen::Matrix<T, 2, 1>& further) const;
};
