/**
 * @file REAudioProvider.h
 *
 * RoboEireann Audio Provider Module Header
 *
 * @author Shauna Recto
 */

#pragma once

#ifdef TARGET_ROBOT
#include <alsa/asoundlib.h>

extern "C" {
#include <WakeupApi.h>
}
#endif

#include "Framework/Module.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include <string>
#include <vector>

MODULE(REAudioProvider,
{,
  REQUIRES(GameState),
  REQUIRES(DamageConfigurationHead),

  REQUIRES(AudioData),
  PROVIDES_WITHOUT_MODIFY(AudioData),
  PROVIDES(FrameInfo),
  LOADS_PARAMETERS(
  {,
    (bool)enabled,
    (unsigned)retries, /* Number of tries to open device. */
    (unsigned)retryDelay, /* Delay before a retry to open device. */
    (unsigned)sampleRate, /* Sample rate to capture on microphone. */
    (unsigned)maxFrames, /* Maximum number of frames read in one tick. */
    (unsigned)tickRate, /* How often you want the audio thread to cycle. */
    (std::string)deviceName, /* The ALSA sound card to be used for audio capture. */
    (std::string)mixerName, /* The ALSA sound card to be used for audio capture. */
    (float)captureVolume, /* The microphone volume in percent. */
    (unsigned)numChannels, /* The number of input channels */
    (unsigned)usedChannels, /* The number of input channels actually used */
  }),
});

class REAudioProvider : public REAudioProviderBase 
{
private:
#ifdef TARGET_ROBOT
  snd_pcm_t* inputDevice = nullptr;
  snd_pcm_hw_params_t* params = nullptr;

  snd_mixer_t* inputMixer = nullptr;
  snd_mixer_selem_id_t* sid = nullptr;
  snd_mixer_elem_t* elem = nullptr;
  float currentCaptureVolume;
  bool bufferInit;

  std::vector<int16_t> audioFramesInt;
  bool setVolume();

  void initBuffers(AudioData& audioData);

/* WakeupAPI related elements */
  const uint wakeupAPIBufferSize = 1024;
  const uint outChannels = 3;
  HWWakeup wakeupHandle = nullptr;
  std::vector<int16_t> waitBuffer; /* Since the API requires 1024, this is needed. */
  std::vector<std::vector<int16_t>> inputAudioBuffers;  /* 4, mic 1, mic 2, mic 3, ref */
  std::vector<std::vector<int16_t>> outputAudioBuffers; /* 4, out 1, out 2, out 3, ref */

  enum AudioInit
  {
    AUDIO_INIT_NOT_STARTED,
    AUDIO_INIT_CTOR,
    AUDIO_INIT_SET_VOLUME,
    AUDIO_INIT_OK,
    AUDIO_INIT_FAILED // sticky state on any failure
  };
  AudioInit initState = AUDIO_INIT_NOT_STARTED;
#endif

  unsigned audioTimestamp = 0;

  void update(AudioData& audioData) override;
  void update(FrameInfo& frameInfo) override { frameInfo.time = audioTimestamp; };

public:
  REAudioProvider();
  ~REAudioProvider();
};
