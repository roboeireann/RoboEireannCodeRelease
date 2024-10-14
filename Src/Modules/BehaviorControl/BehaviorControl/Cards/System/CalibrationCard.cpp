/**
 * @file AutonomousCalibrationCard.cpp
 *
 * This file defines a card that handles the robot control during the autonomous calibration phase.
 *
 * @author Lukas Plecher
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"

#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(CalibrationCard,
{,
  REQUIRES(GameInfo),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) calibrationPhase,
  }),
});

class CalibrationCard : public CalibrationCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.playerMode == GameInfo::calibration;
  }

  bool postconditions() const override
  {
    return theGameInfo.playerMode != GameInfo::calibration;
  }

  void execute() override
  {
    dealer.deal(calibrationPhase)->call();
  }

  void reset() override
  {
    dealer.reset();
  }

  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(CalibrationCard);
