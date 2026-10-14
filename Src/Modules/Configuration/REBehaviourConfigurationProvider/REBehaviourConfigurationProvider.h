/**
 * @file BehaviourConfigurationProvider.h
 * 
 * Load general configurations useful to behaviour and provide them
 * (Based on ConfigurationDataProvider module)
 * 
 * @author Rudi Villing
 */

#pragma once


// #include "Representations/Configuration/BehaviourFormations.h" // deprecated
#include "Representations/Configuration/REFormations.h"
#include "Representations/Configuration/RETactics.h"
#include "Representations/Configuration/REBehaviourParams.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Framework/Module.h"


MODULE(REBehaviourConfigurationProvider,
{,
  REQUIRES(FieldDimensions),

  // PROVIDES(REBehaviourFormations), // deprecated
  PROVIDES(REFormations),
  PROVIDES(RETactics),
  PROVIDES(REBehaviourParams),
});

class REBehaviourConfigurationProvider : public REBehaviourConfigurationProviderBase
{
private:
  static thread_local REBehaviourConfigurationProvider* instance; /**< Points to the only instance of this class in this thread or is nullptr if there is none. */

  // std::unique_ptr<REBehaviourFormations> theBehaviourFormations; // deprecated
  std::unique_ptr<REFormations> theFormations;
  std::unique_ptr<RETactics> theTactics;
  std::unique_ptr<REBehaviourParams> theBehaviourParams;

  // void update(REBehaviourFormations &behaviourFormations) override
  // {
  //   update(behaviourFormations, theBehaviourFormations);
  // }

  void update(REFormations &formations) override { update(formations, theFormations); }

  void update(RETactics &tactics) override { update(tactics, theTactics); }

  void update(REBehaviourParams &behaviourParams) override { update(behaviourParams, theBehaviourParams); }

  // the following functions are templates so that we can easily add additional 
  // behaviour config representations to be added in later

  template<typename T> void update(T& representation, std::unique_ptr<T>& theRepresentation)
  {
    // basically only allow update to copy the representation just once
    if (theRepresentation)
    {
      representation = *theRepresentation;
      theRepresentation = nullptr;
    }
  }

  template<typename T> void read(std::unique_ptr<T>& theRepresentation, const char* fileName = nullptr)
  {
    ASSERT(!theRepresentation);
    theRepresentation = std::make_unique<T>();
    loadModuleParameters(*theRepresentation, TypeRegistry::demangle(typeid(T).name()).c_str(), fileName);
  }

public:
  REBehaviourConfigurationProvider();
  ~REBehaviourConfigurationProvider();
};
