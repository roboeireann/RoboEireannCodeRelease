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


DECL_TLOGGER(tlogger, "BehaviourConfigurationProvider", TextLogging::VERBOSE);


MAKE_MODULE(BehaviourConfigurationProvider, behaviorControl);


thread_local BehaviourConfigurationProvider* BehaviourConfigurationProvider::instance = nullptr;

BehaviourConfigurationProvider::BehaviourConfigurationProvider()
{
  TLOGV(tlogger, "Constructor");
  instance = this;

  read(theBehaviourFormations);
  // theBehaviourFormations->updateAfterRead();
}

BehaviourConfigurationProvider::~BehaviourConfigurationProvider()
{
  instance = nullptr;
}

