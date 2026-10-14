/**
 * @file: SoundSkills.k
 *
 * skills for playing sound and speech
 * (adapted from coderelease2019 Talk.cpp)
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/CoroBehaviourCommon.h"

#include "Platform/SystemCall.h"

#include <string>

namespace RE
{
  struct SoundSkills
  {
    SoundSkills() {}

    void say(const std::string &text)
    {
      if (text != lastThingSaid)
      {
        if (!text.empty())
          SystemCall::say(text);
        lastThingSaid = text;
      }
    }

    void playSound(const std::string &name)
    {
      if (name != lastSoundPlayed)
      {
        if (!name.empty())
          SystemCall::playSound(name);
        lastSoundPlayed = name;
      }
    }

    void reset()
    {
      lastThingSaid.clear();
      lastSoundPlayed.clear();
    }

  private:
    std::string lastThingSaid;
    std::string lastSoundPlayed;
  };

} // namespace RE
