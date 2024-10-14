/**
 * @file AudioProvider.cpp
 * This file implements a module that provides audio samples.
 * @author Thomas Röfer
 * @author Lukas Post
 * @author Lukas Plecher
 * @author Shauna Recto
 */

#include "AudioProvider.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include <type_traits>

#include "Tools/TextLogging.h"

DECL_TLOGGER(tlogger, "AudioProvider", TextLogging::INFO);
MAKE_MODULE(AudioProvider, infrastructure);

#ifdef TARGET_ROBOT

AudioProvider::AudioProvider()
{
  /* Determine how many channels to obtain audio from */
  allChannels ? channels = 4 : channels = 2;

  /* Determine the current PCM Device for audio capture */
  currentPCMDevice = allChannels ? pcmDevice : "default";

  /* Determine the number of mic channels broken */
  unsigned numChannelsBroken = 0;
  for (auto &audioChannelBroken : theDamageConfigurationHead.audioChannelsDefect)
  {
    if (audioChannelBroken)
      numChannelsBroken++;
  }

  /* Check and try to open the capture devices */
  if (numChannelsBroken == channels)
    FAIL("Too many broken microphones to provide audio data");
  else
  {
    /* Try up to max num of retries */
    size_t i;
    for(i = 1; i < retries; i++)
    {
      /* Try to open the device for audio capture */
      if(snd_pcm_open(&handle, currentPCMDevice.c_str(), SND_PCM_STREAM_CAPTURE, 0) >= 0)
      {
        TLOGD(tlogger, "PCM Device opened with {} broken channels...", numChannelsBroken);
        break;
      }
      TLOGD(tlogger, "PCM Device failed to open. Retry {} / {}", i, retries);
      Thread::sleep(retryDelay);
    }
    ASSERT(i < retries);
  }

  /* Setting PCM Device parameters. params does not need to be persistent
   * throughout the program; only at AudioProvider setup.
   */
  snd_pcm_hw_params_t* params;

  VERIFY(snd_pcm_hw_params_malloc(&params) == 0);
  TLOGD(tlogger, "Allocated memory for PCM Device Params");

  VERIFY(snd_pcm_hw_params_any(handle, params) == 0);
  TLOGD(tlogger, "Filled PCM Device Params");

  VERIFY(snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED) == 0);
  TLOGD(tlogger, "Set RW Interleaved PCM Device Access");

  static_assert(std::is_same<AudioData::Sample, short>::value
                || std::is_same<AudioData::Sample, float>::value, "Wrong audio sample type");

  VERIFY(snd_pcm_hw_params_set_format(handle, params, std::is_same<AudioData::Sample, short>::value
                                       ? SND_PCM_FORMAT_S16_LE : SND_PCM_FORMAT_FLOAT_LE) == 0);
  TLOGD(tlogger, "Set Audio Data Format");

  VERIFY(snd_pcm_hw_params_set_rate_near(handle, params, &sampleRate, nullptr) == 0);
  TLOGD(tlogger, "Set Audio Sampling Rate");

  VERIFY(snd_pcm_hw_params_set_channels(handle, params, channels) == 0);
  TLOGD(tlogger, "Set PCM Devices channels, with {} broken", numChannelsBroken);

  VERIFY(snd_pcm_hw_params(handle, params) == 0);
  TLOGD(tlogger, "Set PCM Device Params");

  snd_pcm_hw_params_free(params);
  TLOGD(tlogger, "Params are set free");

  VERIFY(snd_pcm_prepare(handle) == 0);
  TLOGD(tlogger, "Prepared PCM Device interface");

  ASSERT(channels <= 4);

  /* Checks whether the buffer input has been provided a non-negative value */
  std::vector<AudioData::Sample> testBuff;
  testBuff.resize(channels);
  VERIFY(snd_pcm_readi(handle, testBuff.data(), 1) >= 0);
  TLOGD(tlogger, "Valid Audio frames have been obtained");
}

AudioProvider::~AudioProvider()
{
  snd_pcm_close(handle);
}

void AudioProvider::update(AudioData& audioData)
{ 
  /* TODO: Is it possible to do once?
   * Can these lines be moved to constructor?
   * Can constructor receive arguments?
   */
  audioData.channels = channels;
  audioData.sampleRate = sampleRate;

  /* Verifies and corrects current capture volume */
  if(captureVolume != currentCaptureVolume)
  {
    if(!setCaptureVolume(captureCard, captureVolume))
      OUTPUT_WARNING("Could not set capture volume for '" << captureCard << "' to " << captureVolume);
    currentCaptureVolume = captureVolume;
  }

  /* This repeated loop prevents from having the other code from repeating
   * as much as this code block.
   */

  /* If we don't care about audio outside of SET & PLAYING state, clear audio */
  if(onlySoundInSetAndPlaying && theGameInfo.state != STATE_SET && theGameInfo.state != STATE_PLAYING)
  { 
    /* Creates an empty buffer and returns early for this cycle.
    * TODO: Find out how and if it would be desireable to have
    * a completely empty and 0 sized buffer.
    */
    audioData.samples.resize(channels);
    audioData.samples.clear();
    return;
  }
  
  /* Determine the available audio frames obtainable for this cycle 
  * and configures buffer accordingly.
  * 
  * Remember that the samples are interleaved and so must consider this in
  * buffer size.
  */
  unsigned available = std::min(static_cast<unsigned>(snd_pcm_avail(handle)), maxFrames);
  audioData.samples.resize(available * channels);

  /* Reads from the PCM Device buffer and checks whether we have any valid audio. 
  * If not, it then it would proceed to recover the PCM Device status.
  *
  * It then conducts a buffer read test and clears the audio buffer.
  */
  int validAudioFrames = static_cast<int>(snd_pcm_readi(handle, audioData.samples.data(), available));
  if(validAudioFrames < 0)
  {
    OUTPUT_WARNING("Lost audio stream (" << validAudioFrames << "), recovering...");
    snd_pcm_recover(handle, validAudioFrames, 1);  /* Change 1 to 0 for error reason */
    ASSERT(channels <= 4);
    std::vector<AudioData::Sample> testBuff;
    testBuff.resize(channels);
    VERIFY(snd_pcm_readi(handle, testBuff.data(), 1) >= 0);
    audioData.samples.clear();
  }
}

/* Please note that the mixer is different from the PCM Device.
 * This 
 */
bool AudioProvider::setCaptureVolume(const std::string& element, float volumePercent)
{ 
  /* Default values for volume range finding later on. */
  long min = -1, max = -1;

  /* Handy C converted string */
  const char* elemName = element.c_str();

  /* Mixer related objects similar to PCM Device earlier */
  snd_mixer_t *mixer;
  snd_mixer_selem_id_t *sid;

  /* Note: second parameter `int mode` is not used. */
  VERIFY(snd_mixer_open(&mixer, 0) == 0);
  TLOGD(tlogger, "Opened new mixer");

  /* Verify if PCH_input should be put here instead of default */
  VERIFY(snd_mixer_attach(mixer, currentPCMDevice.c_str()) == 0);
  TLOGD(tlogger, "Attached mixer to PCM device");

  /* Register mixer simple element class. 
   *
   * TODO: Check if this function is compatible with the standard mixer
   * interface as this is from the Simple mixer interface.
   */
  VERIFY(snd_mixer_selem_register(mixer, nullptr, nullptr) == 0);
  TLOGD(tlogger, "Registered Mixer Element Class");

  VERIFY(snd_mixer_load(mixer) == 0);
  TLOGD(tlogger, "Loaded mixer classes");

  VERIFY(snd_mixer_selem_id_malloc(&sid) == 0);
  TLOGD(tlogger, "Allocated memory for mixer");

  /* TODO: Check if elemName is valid as "Digital".
   */
  snd_mixer_selem_id_set_index(sid, 0);
  snd_mixer_selem_id_set_name(sid, elemName);

  /* Find Simple Mixer Element and free previous sid allocations */
  snd_mixer_elem_t* elem = snd_mixer_find_selem(mixer, sid);
  snd_mixer_selem_id_free(sid);
  if(!elem)
    return false;

  VERIFY(snd_mixer_selem_get_capture_volume_range(elem, &min, &max) == 0);
  ASSERT(min < max);
  TLOGD(tlogger, "Found volume range");

  /* Calculate the actual volume value */
  volumePercent = Rangef(0.f, 100.f).limit(volumePercent);
  long volume = static_cast<long>(volumePercent * max / 100.f);

  /* Set capture volume for all channels on mixer element */
  VERIFY(snd_mixer_selem_set_capture_volume_all(elem, volume) == 0);
  TLOGD(tlogger, "Set capture volume");

  snd_mixer_close(mixer);
  return true;
}

#else // !defined TARGET_ROBOT

AudioProvider::AudioProvider() {}
AudioProvider::~AudioProvider() {}
void AudioProvider::update(AudioData&) {}

#endif