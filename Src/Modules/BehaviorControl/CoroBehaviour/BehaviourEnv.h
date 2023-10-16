/*
 * @file: BehaviourEnv.h
 *
 * Defines the behaviour environment shared by all behaviour tasks and functions
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Coroutine.h"

// representations used by this class only

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/ActivationGraph.h"

// other includes

#include "Tools/TextLogging.h"

#include <unordered_map>
#include <typeinfo>
#include <typeindex>



namespace CoroBehaviour
{

  /**
   * This class provides the behaviour environment (required/provided
   * representations) to any task that needs it
   */
  struct BehaviourEnv : public CoroEnv
  {
    struct RepresentationRegistry
    {
      void add(const Streamable &representation) 
      { 
        representations.insert({std::type_index(typeid(representation)), &representation}); 

        TLOGD(tlogger(), "add {}", TypeRegistry::demangle(typeid(representation).name()));
      }

      void addModifiable(Streamable &representation)
      { 
        representations.insert({std::type_index(typeid(representation)), &representation});
        modifiables.insert({std::type_index(typeid(representation)), &representation});

        TLOGD(tlogger(), "addModifiable {}", TypeRegistry::demangle(typeid(representation).name()));
      }

    private:
      std::unordered_map<std::type_index, const Streamable*> representations;
      std::unordered_map<std::type_index, Streamable*> modifiables;

      DECL_TLOGGER_FN(tlogger, "BehaviourEnv::RepresentationRegistry", TextLogging::INFO);

      friend struct BehaviourEnv;
    };


    BehaviourEnv() = delete;

    BehaviourEnv(RepresentationRegistry&& registry, ActivationGraph& activationGraph) : registry(registry), 
      // we have to enforce initialization order by placing these here after the registry line
      theGameInfo(get<GameInfo>()),
      theOwnTeamInfo(get<OwnTeamInfo>()),
      theRobotPose(get<RobotPose>()),
      theActivationGraph(activationGraph)
    {
    }

    // representation registry and access, normally not accessed directly but
    // instead via the READS and MODIFIES macros below

    template <class T>
    const T& get()
    {
      auto found = registry.representations.find(std::type_index(typeid(T)));
      if (found == registry.representations.end())
        FAIL(fmt::format("*** Representation {} has not been registered with this BehaviourEnv instance",
                         TypeRegistry::demangle(typeid(T).name())));

      return dynamic_cast<const T&>(*(found->second));
    }

    template <class T>
    T& getModifiable()
    {
      auto found = registry.modifiables.find(std::type_index(typeid(T)));
      if (found == registry.modifiables.end())
        FAIL(fmt::format("*** Modifiable {} has not been registered with this BehaviourEnv instance",
                         TypeRegistry::demangle(typeid(T).name())));

      return dynamic_cast<T&>(*(found->second));
    }

    // some helper functions to make life easier for behaviour

    bool isOurTeamKick() { return theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber; }

    Vector2f toRobotCoordinates(const Vector2f &fieldPoint) { return theRobotPose.inversePose * fieldPoint; }

    Vector2f toFieldCoordinates(const Vector2f &relativePoint) { return theRobotPose * relativePoint; }

  private:
    TextLogger& tlogger = TextLogging::get("BehaviourEnv", TextLogging::DEBUG);

    RepresentationRegistry registry;

    const GameInfo& theGameInfo;
    const OwnTeamInfo& theOwnTeamInfo;
    const RobotPose& theRobotPose;

  public:
    ActivationGraph& theActivationGraph;
  };

} // CoroBehaviour


// Coro behaviours should use these 2 macros to get access to representations they read or modify
// (The representations must have been registered in advance by the BehaviourModule)

/** 
 * in a coro behaviour class, use this to declare that the behaviour will read 
 * the value of some particular representation
 * which must be REQUIRED/USED by the coro behaviour module and registered with
 * this BehaviourEnv
 */
#define READS(Cls)  const Cls& the##Cls { env.get<Cls>() }

/** 
 * in a coro behaviour class, use this to declare that the behaviour will modify 
 * the value of some particular representation
 * which must be PROVIDED by the coro behaviour module and registered with
 * this BehaviourEnv
 */
#define MODIFIES(Cls)  Cls& the##Cls { env.getModifiable<Cls>() }
