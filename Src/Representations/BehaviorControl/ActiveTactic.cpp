/**
 * @file ActiveTactic.cpp
 *
 * Information about the active tactic etc.
 *
 * @author Rudi Villing
 */

#include "Representations/BehaviorControl/ActiveTactic.h"

#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"

void ActiveTactic::draw() const
{
  DECLARE_DEBUG_DRAWING("representation:ActiveTactic", "drawingOnField");

  // ColorRGBA ballColor = ColorRGBA::black.lighter(0.25).alpha(0.5f);
  ColorRGBA ballColor = ColorRGBA::gray.lighter(0.3f).alpha(0.5f);
  ColorRGBA ballBgColor = ColorRGBA::black.alpha(0.2f);
  ColorRGBA posColor = ColorRGBA::black.alpha(0.5f);
  ColorRGBA ballPlayerColor = ColorRGBA::red.alpha(0.3f);

  if (Blackboard::getInstance().exists("FieldDimensions"))
  {
    const FieldDimensions &theFieldDimensions =
        static_cast<const FieldDimensions &>(Blackboard::getInstance()["FieldDimensions"]);

    DRAW_TEXT("representation:ActiveTactic", theFieldDimensions.xPosOwnGroundLine,
              theFieldDimensions.yPosLeftSideline + 220, 150, ColorRGBA::white,
              TypeRegistry::getEnumName(formationId));
  }

  CIRCLE("representation:ActiveTactic", assumedBallPositionOnField.x(), assumedBallPositionOnField.y(), 120, 20, Drawings::dottedPen,
        ballColor, Drawings::solidBrush, ballBgColor);

  for (unsigned i = 0; i < formationPoses.size(); i++)
  {
    const Pose2f& pose = formationPoses[i];
    int playerNumber = Settings::toPlayerNumber(i);

    CIRCLE("representation:ActiveTactic", pose.translation.x(), pose.translation.y(), 150, 30,
            Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

    if (Blackboard::getInstance().exists("GameInfo"))
    {
      const GameInfo &theGameInfo =
          static_cast<const GameInfo &>(Blackboard::getInstance()["GameInfo"]);

      if (playerNumber == theGameInfo.playerNumber) // self?          
        CIRCLE("representation:ActiveTactic", pose.translation.x(), pose.translation.y(), 220, 50,
                Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);
    }

    if (situationRoles[i] == PlayerRole::ballPlayer)
        CIRCLE("representation:ActiveTactic", pose.translation.x(), pose.translation.y(), 270, 30,
                Drawings::solidPen, ballPlayerColor, Drawings::noBrush, ColorRGBA::black);

    DRAW_TEXT("representation:ActiveTactic", pose.translation.x()-75, pose.translation.y()+220, 150,
              posColor, playerNumber);

    Vector2f directionVec = pose * Vector2f(350, 0);

    ARROW("representation:ActiveTactic", pose.translation.x(), pose.translation.y(), directionVec.x(),
          directionVec.y(), 30, Drawings::solidPen, posColor);

    ++playerNumber;
  }

}