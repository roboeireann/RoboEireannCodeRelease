/**
 * @file Threads/Cognition2D.cpp
 *
 * This file implements the execution unit for the cognition thread.
 *
 * @author Jan Fiedler
 * @author Arne Hasselbring
 */

#include "Cognition2D.h"
#include "Modules/Infrastructure/LogDataProvider/LogDataProvider.h"

#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/TeamMessage2023.h"

#include "Tools/Debugging/Debugging.h"
#include "Tools/Settings.h"

REGISTER_EXECUTION_UNIT(Cognition2D)

Cognition2D::Cognition2D()
{
}

bool Cognition2D::beforeFrame()
{
  // read from team comm udp socket
  // theSPLMessageHandler.receive();

  return LogDataProvider::isFrameDataComplete();
}

void Cognition2D::beforeModules()
{
}

void Cognition2D::afterModules()
{
  Blackboard& blackboard = Blackboard::getInstance();

  // NOTE: in the following it seems the representations will exist whether the
  // module that provides them is active or not, so the second check is
  // needed to see which module has actually filled in the representation
  // (sendIfNeeded) in order to decide how to proceed

  if (blackboard.exists("BHumanMessageOutputGenerator") &&
      static_cast<const BHumanMessageOutputGenerator &>(blackboard["BHumanMessageOutputGenerator"]).sendIfNeeded)
  {
    BH_TRACE_MSG("before BHumanMessageOutputGenerator.sendIfNeeded()");
    static_cast<const BHumanMessageOutputGenerator&>(blackboard["BHumanMessageOutputGenerator"]).sendIfNeeded();
  }
  else if (blackboard.exists("TeamMessage2023OutputGenerator") &&
           static_cast<const TeamMessage2023OutputGenerator &>(blackboard["TeamMessage2023OutputGenerator"])
               .sendIfNeeded)
  {
    BH_TRACE_MSG("before TeamMessage2023OutputGenerator.sendIfNeeded()");
    static_cast<const TeamMessage2023OutputGenerator &>(blackboard["TeamMessage2023OutputGenerator"]).sendIfNeeded();
  }
}
