/**
 * @file REBehaviourConfigurationProvider.cpp
 * 
 * Load general configurations useful to behaviour and provide them
 * (Based on ConfigurationDataProvider module)
 * 
 * @author Rudi Villing
 */

#include "REBehaviourConfigurationProvider.h"
// #include "Framework/ModuleContainer.h"
// #include "Framework/Settings.h"
#include "Streaming/InExpr.h"
#include "TextLogging/TextLogging.h"


DECL_TLOGGER(tlogger, "REBehaviourConfigurationProvider", TextLogging::INFO);


MAKE_MODULE(REBehaviourConfigurationProvider);


thread_local REBehaviourConfigurationProvider* REBehaviourConfigurationProvider::instance = nullptr;

REBehaviourConfigurationProvider::REBehaviourConfigurationProvider()
{
  TLOGV(tlogger, "Constructor");
  instance = this;

  // everything in the following {} block potentially makes use of basic arithmetic with 
  // field related symbols in the config file.
  // Only include config files needing these symbols in this block!
  { 
    // prepare field dimension related symbols
    std::unordered_map<std::string, float> fieldSymbols;
    theFieldDimensions.populateSymbolMap(fieldSymbols); // fill in field symbols from theFieldDimensions

    auto loadCfg = [&](const std::string &name, auto &target)
    {
      TLOGI(tlogger, "loading '{}'", name);
      InExprMapFile stream(name, fieldSymbols);
      if (!stream.exists())
        TLOG_ABORT(tlogger, "File '{}' not found in search path", name);
      stream >> target;
      TLOGD(tlogger, "Loaded '{}'", name);
    };
    
    // theBehaviourFormations - DEPRECATED

    // theBehaviourFormations = std::make_unique<BehaviourFormations>();
    // {
    //   InExprMapFile stream("Behaviour/behaviourFormations.cfg", fieldSymbols);
    //   ASSERT(stream.exists());
    //   stream >> *theBehaviourFormations;
    // }

    // theFormations

    theFormations = std::make_unique<REFormations>();
    loadCfg("REBehaviour/REFormations.cfg", *theFormations);

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

    theTactics = std::make_unique<RETactics>();
    loadCfg("REBehaviour/RETactics.cfg", *theTactics);

    theBehaviourParams = std::make_unique<REBehaviourParams>();
    loadCfg("REBehaviour/REBehaviourParams.cfg", *theBehaviourParams);
  }


  // other config files do not use field related symbols and can use default read functionality

}

REBehaviourConfigurationProvider::~REBehaviourConfigurationProvider()
{
  instance = nullptr;
}

