/**
 * @file AudioProvider.h
 * This file declares a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Plecher
 * @author Shauna Recto
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>
#endif
#include "Tools/Module/Module.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include <string>

MODULE(AudioProvider,
{,
  USES(GameInfo),
  REQUIRES(DamageConfigurationHead),
  REQUIRES(ReceivedGameControlData),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  LOADS_PARAMETERS(
  {,
    (unsigned) retries,              /**< Number of tries to open device. */
    (unsigned) retryDelay,           /**< Delay before a retry to open device. */
    (bool) allChannels,              /**< Use all 4 channels, instead of only two*/
    (unsigned) sampleRate,           /**< Sample rate to capture. This variable will contain the frame rate the driver finally selected. */
    (unsigned) maxFrames,            /**< Maximum number of frames read in one cycle. */
    (bool) onlySoundInSetAndPlaying, /**< If true, the module will not provide audio data in game states other than set and playing. */
    (std::string) captureCard,       /**< The ALSA sound card to be used for audio capture. */
    (float) captureVolume,           /**< The microphone volume in percent. */
    (std::string) pcmDevice,         /**< The PCM device configuration (if you plan to use more than 2 mics) */
  }),
});

class AudioProvider : public AudioProviderBase
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* handle;
  unsigned channels;
  float currentCaptureVolume = -1.f;
  std::string currentPCMDevice; 
  bool setCaptureVolume(const std::string& element, float volume);
#endif
  void update(AudioData& audioData) override;

public:
  AudioProvider();
  ~AudioProvider();
};
