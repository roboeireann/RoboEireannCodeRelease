/**
 * @file BehaviourConfigurationProvider.cpp
 * 
 * Load general configurations useful to behaviour and provide them
 * (Based on ConfigurationDataProvider module)
 * 
 * @author Rudi Villing
 */

#include "BehaviourConfigurationProvider.h"
#include "Tools/Framework/ModuleContainer.h"
#include "Tools/Settings.h"
#include "Tools/TextLogging.h"
#include "Tools/Streams/InExpr.h"


DECL_TLOGGER(tlogger, "BehaviourConfigurationProvider", TextLogging::INFO);


MAKE_MODULE(BehaviourConfigurationProvider, behaviorControl);


thread_local BehaviourConfigurationProvider* BehaviourConfigurationProvider::instance = nullptr;

BehaviourConfigurationProvider::BehaviourConfigurationProvider()
{
  TLOGV(tlogger, "Constructor");
  instance = this;

  // everything in the following {} block potentially makes use of basic arithmetic with 
  // field related symbols in the config file.
  // Only include config files needing these symbols in this block!
  { 
    // prepare field dimension related symbols
    std::unordered_map<std::string, float> fieldSymbols;
    theFieldDimensions.populateSymbols(fieldSymbols); // fill in field symbols from theFieldDimensions

    // theBehaviourFormations - DEPRECATED

    theBehaviourFormations = std::make_unique<BehaviourFormations>();
    {
      InExprMapFile stream("Behaviour/behaviourFormations.cfg", fieldSymbols);
      ASSERT(stream.exists());
      stream >> *theBehaviourFormations;
    }

    // theFormations 

    theFormations = std::make_unique<Formations>();
    {
      InExprMapFile stream("Behaviour/formations.cfg", fieldSymbols);
      ASSERT(stream.exists());
      stream >> *theFormations;

      TLOGD(tlogger, "Loaded formations");
    }

    // theKickoffTactics

//     theKickoffTactics = std::make_unique<KickoffTactics>();
//     {
//       InExprMapFile stream("Behaviour/kickoffTactics.cfg", fieldSymbols);
//       ASSERT(stream.exists());
//       stream >> *theKickoffTactics;
// 
//       TLOGD(tlogger, "Loaded kickoffTactics");
//     }

    // theTactics depends on all the previous representations

    theTactics = std::make_unique<Tactics>();
    {
      InExprMapFile stream("Behaviour/tactics.cfg", fieldSymbols);
      ASSERT(stream.exists());
      stream >> *theTactics;

      TLOGD(tlogger, "Loaded tactics");
    }
  }

  // other config files do not use field related symbols and can use default read functionality

  read(theBehaviourParams, "Behaviour/behaviourParams.cfg");
}

BehaviourConfigurationProvider::~BehaviourConfigurationProvider()
{
  instance = nullptr;
}

