/**
 * @file FieldDimensions.h
 *
 * Description of the dimensions of the field.
 *
 * @author Matthias Jüngel
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Boundary.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Streams/EnumIndexedArray.h"

#include <unordered_map>

STREAMABLE(SimpleFieldDimensions,
{,
  (float) xPosOpponentFieldBorder,
  (float) xPosOpponentGoal,
  (float) xPosOpponentGoalPost,
  (float) xPosOpponentGroundLine,
  (float) xPosOpponentPenaltyArea,
  (float) xPosOpponentPenaltyMark,
  (float) xPosPenaltyStrikerStartPosition,
  (float) xPosHalfWayLine,
  (float) xPosOwnPenaltyArea,
  (float) xPosOwnPenaltyMark,
  (float) xPosOwnGroundLine,
  (float) xPosOwnGoalPost,
  (float) xPosOwnGoal,
  (float) xPosOwnFieldBorder,
  (float) xPosOpponentGoalArea,
  (float) xPosOwnGoalArea,

  (float) yPosLeftFieldBorder,
  (float) yPosLeftSideline,
  (float) yPosLeftPenaltyArea,
  (float) yPosLeftGoal,
  (float) yPosCenterGoal,
  (float) yPosRightGoal,
  (float) yPosRightPenaltyArea,
  (float) yPosRightSideline,
  (float) yPosRightFieldBorder,
  (float) yPosLeftGoalArea,
  (float) yPosRightGoalArea,

  //other dimensions
  (float) fieldLinesWidth,
  (float) centerCircleRadius,
  (float) goalPostRadius,
  (float) crossBarRadius,
  (float) goalHeight,
  (float) penaltyMarkSize, //vertical (and horizontal) size of a penaltyMark
});

/**
 * Class containing definitions and functions
 * regarding field dimensions.
 *
 * @author Max Risler
 */
struct FieldDimensions : public SimpleFieldDimensions
{
  /**
   * This is a collection of line- or boundary segments with start-Pose2f and length.
   */
  struct LinesTable
  {
    STREAMABLE(Line,
    {
      bool isPartOfCircle = false, /**< Whether the line is a part of a circle. */

      (Vector2f)(Vector2f::Zero()) from, /**< Begin of the line. */
      (Vector2f)(Vector2f::Zero()) to, /**< End of the line. */
    });

    STREAMABLE(Circle,
    {,
      (Vector2f) center, /**< The center of the circle. */
      (float) radius, /**< The radius of the circle. */
      (int) numOfSegments, /**< The number of segments used to discretize the circle. */
    });

    std::vector<Line> lines;

    void push(const Pose2f& p, float l, bool isPartOfCircle = false);
    void push(const Vector2f& s, const Vector2f& e, bool isPartOfCircle = false);
    void pushCircle(const Vector2f& center, float radius, int numOfSegments);

  };

  /**
   * All different corner classes.
   */
  ENUM(CornerClass,
  {,
    xCorner,
    tCorner0,
    tCorner90,
    tCorner180,
    tCorner270,
    lCorner0,
    lCorner90,
    lCorner180,
    lCorner270,
  });
  static const int numOfCornerClasses = numOfCornerClasss; // extra, because numOfCornerClasss isn't so nice

private:
  LinesTable straightFieldLines; /**< The field lines as read from the stream. */
  LinesTable::Circle centerCircle; /**< The center circle as read from the stream. */

public:
  Boundaryf boundary; ///< The outer boundary of the field.
  LinesTable fieldLines; ///< Table of line segments
  LinesTable goalFrameLines; ///< Table of line segments that contains the parts of the goal frame that are on the ground.
  LinesTable fieldLinesWithGoalFrame; ///< Table of line segments that contains both fieldLines and goalFrameLines
  ENUM_INDEXED_ARRAY(std::vector<Vector2f>, CornerClass) corners; ///< All corners on the field.

  /**
   * Read field dimensions from configuration file.
   */
  void load();

  /**
   * Returns true when p is inside the carpet.
   */
  bool isInsideCarpet(const Vector2f& p) const
  {
    return p.x() <= xPosOpponentFieldBorder && p.x() >= xPosOwnFieldBorder && p.y() <= yPosLeftFieldBorder && p.y() >= yPosRightFieldBorder;
  }

  /**
   * The function clips a point to the carpet.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToCarpet(Vector2f& v) const
  {
    const Vector2f old = v;
    if(v.x() > xPosOpponentFieldBorder)
      v.x() = xPosOpponentFieldBorder;
    else if(v.x() < xPosOwnFieldBorder)
      v.x() = xPosOwnFieldBorder;
    if(v.y() > yPosLeftFieldBorder)
      v.y() = yPosLeftFieldBorder;
    else if(v.y() < yPosRightFieldBorder)
      v.y() = yPosRightFieldBorder;
    return (v - old).norm();
  }

  /**
   * Returns true when p is inside the playing field.
   */
  bool isInsideField(const Vector2f& p) const
  {
    return p.x() <= xPosOpponentGroundLine && p.x() >= xPosOwnGroundLine && p.y() <= yPosLeftSideline && p.y() >= yPosRightSideline;
  }

  /**
   * Returns true when p is inside the opponent goal.
   * (Allows for optional tolerance)
   */
  bool isInsideOpponentGoal(const Vector2f& p, const Vector2f& tolerance = Vector2f(70.f, 100.f)) const
  {
    return (p.x() >= (xPosOpponentGroundLine + tolerance.x())) && // slightly inside goal (so we don't leave a ball sit on the line)
           (p.y() < (yPosLeftGoal + tolerance.y())) && // slightly wider than real goal
           (p.y() > (yPosRightGoal - tolerance.y()));
  }

  /**
  * Returns true when p is inside the own half.
  */
  bool isInsideOwnHalf(const Vector2f& p) const
  {
    return p.x() <= xPosHalfWayLine && p.x() >= xPosOwnGroundLine && p.y() <= yPosLeftSideline && p.y() >= yPosRightSideline;
  }

  /**
   * The function clips a point to the field.
   * @param v The point.
   * @return How far was the point moved?
   */
  float clipToField(Vector2f& v) const
  {
    const Vector2f old = v;
    if(v.x() > xPosOpponentGroundLine)
      v.x() = xPosOpponentGroundLine;
    else if(v.x() < xPosOwnGroundLine)
      v.x() = xPosOwnGroundLine;
    if(v.y() > yPosLeftSideline)
      v.y() = yPosLeftSideline;
    else if(v.y() < yPosRightSideline)
      v.y() = yPosRightSideline;
    return (v - old).norm();
  }

  /**
   * The function returns a random pose inside the field.
   * @return The random pose.
   */
  Pose2f randomPoseOnField() const;

  /**
   * The function returns a random pose on the carpet.
   * @return The random pose.
   */
  Pose2f randomPoseOnCarpet() const;

  /**
   * The method draws the field lines.
   */
  void draw() const;

  /**
   * Draws the goal frame.
   */
  void drawGoalFrame() const;


  /**
   * add symbols to a map so they can be used in other config files
   */
  void populateSymbols(std::unordered_map<std::string, float>& symbols) const;

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override;

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override;

private:
  static void reg();

  /**
   * The method draws the field polygons.
   */
  void drawPolygons() const;

  /**
   * The method draws the field lines.
   */
  void drawLines() const;

  /**
   * The method draws the field lines.
   */
  void drawCorners() const;
};
