/**
 * @file IntersectionsProvider.cpp
 */

#include "IntersectionsProvider.h"
#include "Debugging/DebugDrawings.h"
#include "Math/Geometry.h"
#include "Tools/Math/Transformation.h"

MAKE_MODULE(IntersectionsProvider);

void IntersectionsProvider::update(IntersectionsPercept& intersectionsPercept)
{
  intersectionsPercept.intersections.clear();

  DECLARE_DEBUG_DRAWING("module:IntersectionsProvider:field", "drawingOnField");
  DECLARE_DEBUG_DRAWING("module:IntersectionsProvider:image", "drawingOnImage");

  for(unsigned i = 0; i < theLinesPercept.lines.size(); ++i)
  {
    if(theLinesPercept.lines[i].belongsToCircle)
      continue;

    for(unsigned j = i + 1; j < theLinesPercept.lines.size(); ++j)
    {
      if(theLinesPercept.lines[j].belongsToCircle)
        continue;

      Vector2f intersection;
      if(!Geometry::getIntersectionOfLines(theLinesPercept.lines[i].line, theLinesPercept.lines[j].line, intersection))
        continue;

      const float dot = theLinesPercept.lines[i].line.direction.normalized().dot(
                          theLinesPercept.lines[j].line.direction.normalized());
      if(std::abs(std::acos(std::clamp(dot, -1.f, 1.f)) - pi_2) > maxAllowedAngleDifference)
        continue;

      Vector2f intersectionInImage;
      if(!Transformation::robotToImage(intersection, theCameraMatrix, theCameraInfo, intersectionInImage) ||
         intersectionInImage.x() < 0 || intersectionInImage.x() >= theCameraInfo.width ||
         intersectionInImage.y() < 0 || intersectionInImage.y() >= theCameraInfo.height)
        continue;

      const LinesPercept::Line& line1 = theLinesPercept.lines[i];
      const LinesPercept::Line& line2 = theLinesPercept.lines[j];

      Vector2f line1Further, line2Further;
      const float line1Dist = getCloserPoint(line1.firstField, line1.lastField, intersection, line1Further);
      const float line2Dist = getCloserPoint(line2.firstField, line2.lastField, intersection, line2Further);

      const bool line1OnSegment = isPointInSegment(line1, intersection);
      const bool line2OnSegment = isPointInSegment(line2, intersection);
      const bool line1Ends = maxIntersectionGap > line1Dist;
      const bool line2Ends = maxIntersectionGap > line2Dist;

      if((!line1OnSegment && !line1Ends) || (!line2OnSegment && !line2Ends))
        continue;

      Vector2f pos;
      Matrix2f cov;
      theMeasurementCovariance.transformPointWithCov(intersectionInImage, 0.f, pos, cov);

      IntersectionsPercept::Intersection::IntersectionType type;
      Vector2f dir1, dir2;
      unsigned line1Idx = i, line2Idx = j;

      if(line1Ends && line2Ends)
      {
        type = IntersectionsPercept::Intersection::L;
        dir1 = line1Further - intersection;
        dir2 = line2Further - intersection;
      }
      else if(!line1Ends && !line2Ends)
      {
        type = IntersectionsPercept::Intersection::X;
        dir1 = line1.line.direction;
        dir2 = line2.line.direction;
      }
      else
      {
        type = IntersectionsPercept::Intersection::T;
        const bool line1IsStem = line1Ends;
        Vector2f vertical   = line1IsStem ? (line1Further - intersection) : (line2Further - intersection);
        Vector2f horizontal = line1IsStem ? (line2Further - intersection) : (line1Further - intersection);
        enforceTIntersectionDirections(vertical, horizontal);
        dir1 = vertical;
        dir2 = horizontal;
        if(!line1IsStem)
          std::swap(line1Idx, line2Idx);
      }

      intersectionsPercept.intersections.emplace_back(type, pos, cov, dir1, dir2, line1Idx, line2Idx);

      COMPLEX_DRAWING("module:IntersectionsProvider:field")
      {
        const Vector2f fieldCoords = theRobotPose * pos;
        DRAW_TEXT("module:IntersectionsProvider:field", fieldCoords.x() + 5, fieldCoords.y() + 5, 10,
                  ColorRGBA::black, std::string(1, static_cast<char>(*(TypeRegistry::getEnumName(type)))));
        CROSS("module:IntersectionsProvider:field", fieldCoords.x(), fieldCoords.y(), 60, 30,
              Drawings::solidPen, ColorRGBA::blue);
      }
      COMPLEX_DRAWING("module:IntersectionsProvider:image")
      {
        DRAW_TEXT("module:IntersectionsProvider:image", intersectionInImage.x() + 15, intersectionInImage.y() + 15, 10,
                  ColorRGBA::black, std::string(1, static_cast<char>(*(TypeRegistry::getEnumName(type)))));
        CROSS("module:IntersectionsProvider:image", intersectionInImage.x(), intersectionInImage.y(), 5, 3,
              Drawings::solidPen, ColorRGBA::blue);
      }
    }
  }
}

void IntersectionsProvider::enforceTIntersectionDirections(const Vector2f& vertical, Vector2f& horizontal) const
{
  Vector2f vertical90 = vertical;
  vertical90.rotate(pi_2);
  if(vertical90.angleTo(horizontal) > pi_2)
    horizontal.mirror();
}

template<typename T>
float IntersectionsProvider::getCloserPoint(const Eigen::Matrix<T, 2, 1>& a, const Eigen::Matrix<T, 2, 1>& b,
                                            const Eigen::Matrix<T, 2, 1>& target,
                                            Eigen::Matrix<T, 2, 1>& further) const
{
  const float aDist = Vector2f(static_cast<float>(a.x() - target.x()), static_cast<float>(a.y() - target.y())).squaredNorm();
  const float bDist = Vector2f(static_cast<float>(b.x() - target.x()), static_cast<float>(b.y() - target.y())).squaredNorm();
  if(aDist < bDist) { further = b; return std::sqrt(aDist); }
  further = a; return std::sqrt(bDist);
}

bool IntersectionsProvider::isPointInSegment(const LinesPercept::Line& line, const Vector2f& point) const
{
  const float lineSquaredNorm = (line.firstField - line.lastField).squaredNorm();
  return (line.firstField - point).squaredNorm() <= lineSquaredNorm &&
         (line.lastField  - point).squaredNorm() <= lineSquaredNorm;
}
