/**
 * @file TeammatesLocationModel.cpp
 * 
 * Representation of best known teammate positions based on communicated
 * info and anything we know about their typical behaviour (normally just
 * based on the expected tactic pose)
 * 
 * @author Rudi Villing
 */

#include "TeammatesLocationModel.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Math/Approx.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Modeling/Obstacle.h"
#include "BallModel.h"
#include "RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"

void TeammatesLocationModel::draw() const
{
  DEBUG_DRAWING3D("representation:TeammatesLocationModel", "field")
  {
    ColorRGBA posColor = ColorRGBA::violet.darker(0.4f).alpha(0.4f);

    // draw robots
    for(const auto& location : locations)
    {
      CYLINDER3D("representation:TeammatesLocationModel", 
                 location.pose.translation.x(), location.pose.translation.y(), 0.0f, 
                 0.0f, 0.0f, 0.0f, 80.0f, 20.0f, posColor);

      Vector3f center3d = Vector3f(location.pose.translation.x(), location.pose.translation.y(), 0);
      Vector2f direction2d = location.pose * Vector2f(200, 0);
      Vector3f direction3d = Vector3f(direction2d.x(), direction2d.y(), 0);

      CYLINDERARROW3D("representation:TeammatesLocationModel", center3d, direction3d, 10.f, 45.f, 45.f, posColor);
    }
  }

  DEBUG_DRAWING("representation:TeammatesLocationModel", "drawingOnField")
  {
    ColorRGBA posColor = ColorRGBA::violet.darker(0.4f).alpha(0.4f);

    for(const auto& location : locations)
    {
      CIRCLE("representation:TeammatesLocationModel", location.pose.translation.x(), location.pose.translation.y(), 
             150, 20, Drawings::solidPen, posColor, Drawings::solidBrush, posColor);
      CROSS("representation:TeammatesLocationModel", location.pose.translation.x(), location.pose.translation.y(), 
            20, 40, Drawings::solidPen, ColorRGBA::white);

      if (location.locationType == independent)
        CIRCLE("representation:TeammatesLocationModel", location.pose.translation.x(), location.pose.translation.y(),
               200, 40, Drawings::dottedPen, posColor, Drawings::noBrush, ColorRGBA::black);

      Vector2f directionVec = location.pose * Vector2f(300, 0);

      ARROW("representation:TeammatesLocationModel", location.pose.translation.x(), location.pose.translation.y(), 
            directionVec.x(), directionVec.y(), 20, Drawings::solidPen, posColor);
    }
  }
}
