/**
 * @file REStandStillCard.cpp
 *
 * This file implements nothing.
 * (Based on Code release position for kickoff)
 *
 * @author Rudi Villing
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(REStandStillCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
});

class REStandStillCard : public REStandStillCardBase
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
    theActivitySkill(BehaviorStatus::unknown);
    theLookForwardSkill();
    theStandSkill();
  }
};

MAKE_CARD(REStandStillCard);
