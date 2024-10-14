/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldFeatureOverview.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"

#define PLOT_SINGE_TSL(name) \
  PLOT("representation:FieldFeatureOverview:timeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));

void FieldFeatureOverview::draw() const
{
  if(Blackboard::getInstance().exists("FrameInfo"))
  {
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);

    PLOT_SINGE_TSL(penaltyArea);
    PLOT_SINGE_TSL(midCircle);
    PLOT_SINGE_TSL(penaltyMarkWithPenaltyAreaLine);

    PLOT("representation:FieldFeatureOverview:timeSinceLast", theFrameInfo.getTimeSince(combinedStatus.lastSeen));
  }
}
