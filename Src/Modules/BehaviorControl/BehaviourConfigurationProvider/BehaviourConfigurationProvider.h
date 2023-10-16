/**
 * @file BehaviourConfigurationProvider.h
 * 
 * Load general configurations useful to behaviour and provide them
 * (Based on ConfigurationDataProvider module)
 * 
 * @author Rudi Villing
 */

#pragma once


#include "Representations/Configuration/BehaviourFormations.h"
#include "Tools/Module/Module.h"


MODULE(BehaviourConfigurationProvider,
{,
  PROVIDES(BehaviourFormations),
});

class BehaviourConfigurationProvider : public BehaviourConfigurationProviderBase
{
private:
  static thread_local BehaviourConfigurationProvider* instance; /**< Points to the only instance of this class in this thread or is nullptr if there is none. */

  std::unique_ptr<BehaviourFormations> theBehaviourFormations;

  void update(BehaviourFormations& behaviourFormations) override { update(behaviourFormations, theBehaviourFormations); }

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
