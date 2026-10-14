/**
 * @file: SoundSkills.k
 *
 * skills for playing sound and speech
 * Adapted from BH2025 Audio.cpp
 *
 * @author Arne Hasselbring
 * @author Jan Blumenkamp
 * @author: Rudi Villing (modifications)
 */

#include "REHybridBehaviourModule.h"


HSM_DEFINE((BehaviourContainer, playSoundTask),
       ARGS((const std::string&, soundName),
            (bool force)),
       VARS((std::string, lastSoundEnqueued)))
{
  HSM_INITIAL_STATE(done)
  {
    HSM_TRANSITIONS
    {
      if (args.soundName != lastSoundEnqueued)
        HSM_GOTO(execute);
    }
  }

  HSM_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (args.soundName == lastSoundEnqueued)
        HSM_GOTO(done);
    }

    SystemCall::playSound(args.soundName, args.force);
    lastSoundEnqueued = args.soundName;
  }
}

HSM_DEFINE((BehaviourContainer, sayTask),
       ARGS((const std::string&, text),
            (bool, force),
            (float, speed)),
       VARS((std::string, lastTextEnqueued)))
{
  HSM_INITIAL_STATE(done)
  {
    HSM_TRANSITIONS
    {
      if (args.text != lastTextEnqueued)
        HSM_GOTO(execute);
    }
  }

  HSM_STATE(execute)
  {
    HSM_TRANSITIONS
    {
      if (args.text == lastTextEnqueued)
        HSM_GOTO(done);
    }
    SystemCall::say(args.text, args.force, args.speed);
    lastTextEnqueued = args.text;
  }
}
