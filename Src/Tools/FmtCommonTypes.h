/**
 * @file FmtCommonTypes.h
 *
 * This file declares custom formatters for some of our common types
 *
 * @author Rudi Villing
 */

#pragma once

#include "FmtFormatter.h"

#include "Math/Eigen.h"
#include "Math/Angle.h"
#include "Math/Pose2f.h"

#include "fmt/core.h"

/**
 * wrapper type to enable auto-formatting of angles and get around the implicit conversion of Angle to float
 * Also works if the angles are already floats
 */
struct FmtAngle
{
  float angle;
  FmtAngle(float a) : angle(a) {}
};

FMT_FORMATTER(FmtAngle, fa, "{:.1f}_deg", toDegrees(fa.angle));  /// usage: FmtAngle(someAngleOrFloatInRadians)
FMT_FORMATTER(Angle::Fmt, anglefmt, "{:.1f}_deg", toDegrees(anglefmt.value)); /// usage: someAngle.fmt() where someAngle is an Angle

FMT_FORMATTER(Vector2f, v, "{{{:.0f}, {:.0f}}}", v.x(), v.y());
FMT_FORMATTER(Vector2i, v, "{{{:d}, {:d}}}", v.x(), v.y());

FMT_FORMATTER(Pose2f, p, "{{{:.1f}_deg, {:.0f}, {:.0f}}}", p.rotation.toDegrees(), p.translation.x(), p.translation.y());
