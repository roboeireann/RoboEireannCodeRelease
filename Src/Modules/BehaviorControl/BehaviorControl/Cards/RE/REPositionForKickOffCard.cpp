/**
 * @file REPositionForKickOffCard.cpp
 *
 * This file implements nothing.
 *
 * @author Arne Hasselbring
 * @author Rudi Villing
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(REPositionForKickOffCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
});

class REPositionForKickOffCard : public REPositionForKickOffCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::rePositionForKickOff);
    theLookForwardSkill();
    theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("Please implement a behavior for me!");
  }
};

MAKE_CARD(REPositionForKickOffCard);
