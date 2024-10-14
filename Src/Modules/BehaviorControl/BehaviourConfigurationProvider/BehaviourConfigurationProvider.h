/**
 * @file BehaviourConfigurationProvider.h
 * 
 * Load general configurations useful to behaviour and provide them
 * (Based on ConfigurationDataProvider module)
 * 
 * @author Rudi Villing
 */

#pragma once


#include "Representations/Configuration/BehaviourFormations.h" // deprecated
#include "Representations/Configuration/Formations.h"
#include "Representations/Configuration/Tactics.h"
#include "Representations/Configuration/BehaviourParams.h"
#include "Representations/Configuration/FieldDimensions.h"

#include "Tools/Module/Module.h"


MODULE(BehaviourConfigurationProvider,
{,
  REQUIRES(FieldDimensions),

  PROVIDES(BehaviourFormations), // deprecated
  PROVIDES(Formations),
  PROVIDES(Tactics),
  PROVIDES(BehaviourParams),
});

class BehaviourConfigurationProvider : public BehaviourConfigurationProviderBase
{
private:
  static thread_local BehaviourConfigurationProvider* instance; /**< Points to the only instance of this class in this thread or is nullptr if there is none. */

  std::unique_ptr<BehaviourFormations> theBehaviourFormations; // deprecated
  std::unique_ptr<Formations> theFormations;
  std::unique_ptr<Tactics> theTactics;
  std::unique_ptr<BehaviourParams> theBehaviourParams;

  void update(BehaviourFormations &behaviourFormations) override
  {
    update(behaviourFormations, theBehaviourFormations);
  }

  void update(Formations &formations) override
  {
    update(formations, theFormations);
  }

  void update(Tactics &tactics) override
  {
    update(tactics, theTactics);
  }

  void update(BehaviourParams &behaviourParams) override { update(behaviourParams, theBehaviourParams); }

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
  BehaviourConfigurationProvider();
  ~BehaviourConfigurationProvider();
};
