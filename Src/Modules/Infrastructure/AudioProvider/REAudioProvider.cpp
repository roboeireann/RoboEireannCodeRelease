/**
 * @file REAudioProvider.cpp
 *
 * RoboEireann Audio Provider Module Source
 *
 * @author Shauna Recto
 */

#include "REAudioProvider.h"
#include "Platform/Thread.h"
#include "Platform/Time.h"
#include "Platform/SystemCall.h"
#include "TextLogging/TextLogging.h"

MAKE_MODULE(REAudioProvider);

DECL_TLOGGER(tlogger, "REAudioProvider", TextLogging::INFO);


#define CHECK_ALSA(alsaFn, errorAction)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    int err_ = (alsaFn);                                                                                               \
    if (err_ < 0)                                                                                                      \
    {                                                                                                                  \
      initState = AUDIO_INIT_FAILED;                                                                                   \
      TLOGW(tlogger, "{} failed: {} ({})", #alsaFn, snd_strerror(err_), err_);                                         \
      SystemCall::playSound("doh.wav");                                                                                \
      SystemCall::say("Audio initialization failed");                                                                  \
      errorAction;                                                                                                     \
    }                                                                                                                  \
  } while (0)


REAudioProvider::REAudioProvider()
{
#ifdef TARGET_ROBOT

  if (!enabled)
    return;

  OUTPUT_TEXT("Initializing audio capture device...");
  TLOGI(tlogger, "Ctor: Initializing audio capture device...");

  /* Check for whether we can open the input audio device */
  unsigned retryNumber;
  int err;
  for (retryNumber = 0; retryNumber < retries; retryNumber++)
  {
    err = snd_pcm_open(&inputDevice, deviceName.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err == 0)
      break;
    else
    {
      TLOGW(tlogger, "Failed to open microphone (attempt {}): {} ({})", retryNumber+1, snd_strerror(err), err);
      Thread::sleep(retryDelay);
    }
  }
  if (retryNumber == retries) // did we use up all retries?
  {
    initState = AUDIO_INIT_FAILED;
    return;
  }


  currentCaptureVolume = 100.f;

  /* Static Allocation to reduce malloc issues */
  snd_pcm_hw_params_alloca(&params);
  snd_pcm_hw_params_any(inputDevice, params);

  snd_pcm_hw_params_set_access(inputDevice, params, SND_PCM_ACCESS_RW_INTERLEAVED);
  snd_pcm_hw_params_set_format(inputDevice, params, SND_PCM_FORMAT_S16_LE);
  snd_pcm_hw_params_set_channels(inputDevice, params, numChannels);
  snd_pcm_hw_params_set_rate_near(inputDevice, params, &sampleRate, nullptr);

  CHECK_ALSA(snd_pcm_hw_params(inputDevice, params), return);

  OUTPUT_TEXT("Starting audio device...");
  
  CHECK_ALSA(snd_pcm_prepare(inputDevice), return);
  CHECK_ALSA(snd_pcm_start(inputDevice), return);

  initState = AUDIO_INIT_CTOR;  

  bufferInit = false;
  setVolume();
#endif
}

REAudioProvider::~REAudioProvider()
{
#ifdef TARGET_ROBOT
  if (!enabled)
    return;

  Duerwen_wakeup_unit(wakeupHandle);
  if (inputMixer)
    snd_mixer_close(inputMixer);
#endif
}

#ifdef TARGET_ROBOT
bool REAudioProvider::setVolume()
{
  if (!enabled || initState == AUDIO_INIT_FAILED)
    return false;

  if (mixerName == "")
  {
    OUTPUT_WARNING("There is no mixer. volume not set. Expect 100%...");
    return false;
  }

  CHECK_ALSA(snd_mixer_open(&inputMixer, 0), return false);
  CHECK_ALSA(snd_mixer_attach(inputMixer, deviceName.c_str()), return false);

  CHECK_ALSA(snd_mixer_selem_register(inputMixer, nullptr, nullptr), return false);
  CHECK_ALSA(snd_mixer_load(inputMixer), return false);

  snd_mixer_selem_id_alloca(&sid);
  snd_mixer_selem_id_set_index(sid, 0);
  snd_mixer_selem_id_set_name(sid, mixerName.c_str());

  elem = snd_mixer_find_selem(inputMixer, sid);
  if (!elem)
  {
    initState = AUDIO_INIT_FAILED;
    TLOGW(tlogger, "snd_mixer_find_selem failed: no matching mixer element '{}'", mixerName);
    return false;
  }

  long minVol = -1;
  long maxVol = -1;
  CHECK_ALSA(snd_mixer_selem_get_capture_volume_range(elem, &minVol, &maxVol), return false);
  if (!(minVol < maxVol))
  {
    initState = AUDIO_INIT_FAILED;
    TLOGW(tlogger, "snd_mixer_selem_get_capture_volume_range failed: {} < {} is false", minVol, maxVol);
    return false;
  }

  currentCaptureVolume = std::clamp(captureVolume, 0.f, 100.f);
  long volume = static_cast<long>(currentCaptureVolume * maxVol / 100.f);

  CHECK_ALSA(snd_mixer_selem_set_capture_volume_all(elem, volume), return false);
  
  initState = AUDIO_INIT_SET_VOLUME;
  return true;
}

// void REAudioProvider::writeFloatSamples(AudioData& audioData)
// {
//   for (uint sampleIndex = 0; sampleIndex < audioData.samplesCaptured * audioData.channels; sampleIndex++)
//     audioData.samples[sampleIndex] = static_cast<float>(audioFramesInt[sampleIndex]) / INT16_MAX;
// }


void REAudioProvider::initBuffers(AudioData& audioData)
{
  int retDuerwen = -1;
  constexpr int maxRetries = 10;
  int initRetries;
  for (initRetries = 0; initRetries < maxRetries; initRetries++)
  {
    OUTPUT_TEXT("Initializing WakeupAPI...");
    TLOGI(tlogger, "Initializing WakeupAPI (attempt {})...", initRetries+1);
    
    retDuerwen = Duerwen_wakeup_init(&wakeupHandle, 0);
    if (retDuerwen == 0)
      break;
    else
    {
      if (wakeupHandle != nullptr)
      {
        Duerwen_wakeup_unit(wakeupHandle);
        wakeupHandle = nullptr;
      }
      Thread::sleep(100);
    }
  }
  if (initRetries == maxRetries)
  {
    TLOGW(tlogger, "Failed to initialize WakeupAPI - giving up");
    initState = AUDIO_INIT_FAILED;
    return;
  }

  // VERIFY(retDuerwen == 0);

  OUTPUT_TEXT("Initializing audio buffers...");

  audioFramesInt.resize(maxFrames * numChannels);
  audioData.samples.reserve(maxFrames * outChannels);
  std::fill(audioFramesInt.begin(), audioFramesInt.end(), 0);
  // std::fill(audioData.samples.begin(), audioData.samples.end(), 0);

  audioData.channels = outChannels;
  audioData.sampleRate = sampleRate;

  inputAudioBuffers.resize(usedChannels);
  outputAudioBuffers.resize(usedChannels);

  for (size_t bufferInd = 0; bufferInd < inputAudioBuffers.size(); bufferInd++)
  {
    inputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
    outputAudioBuffers[bufferInd].resize(wakeupAPIBufferSize);
  }

  initState = AUDIO_INIT_OK;
  TLOGI(tlogger, "WakeupAPI initialized");
}
#endif

void REAudioProvider::update([[maybe_unused]] AudioData& audioData)
{
#ifdef TARGET_ROBOT
  audioData.samples.clear(); // size is now zero

  if (!enabled || initState == AUDIO_INIT_FAILED)
  {
    Thread::sleep(30);
    return;
  }

  /* Create our audio buffers and clear values to 0 */
  if (!bufferInit)
  {
    initBuffers(audioData); // note: might block for a period of time retrying this
    if (initState == AUDIO_INIT_FAILED)
      return;

    bufferInit = true;
  }

  // use the following loop to ensure that the update function generally doesn't return
  // until audioData contains usable samples

  unsigned startMs = Time::getCurrentSystemTime();
  for (; Time::getTimeSince(startMs) < 256; ) // 256 ms is 4 times as long as the expected time to receive 1024 samples at 16000Hz
  {
    snd_pcm_sframes_t availableFrames = snd_pcm_avail(inputDevice);

    if (availableFrames < 0)
    {
      OUTPUT_WARNING("Error fetch available frames, recovering...");
      snd_pcm_recover(inputDevice, static_cast<int>(availableFrames), 0);
      Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
      break;
    }
    else // available frames OK
    {
      unsigned numReadableFrames = std::min(static_cast<unsigned>(availableFrames), maxFrames);
      int numAudioFrames = static_cast<int>(snd_pcm_readi(inputDevice, audioFramesInt.data(), numReadableFrames));

      // Try to recover the audio device
      if (numAudioFrames < 0)
      {
        OUTPUT_WARNING("Audio record error, recovering...");
        numAudioFrames = snd_pcm_recover(inputDevice, numAudioFrames, 0);
        Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        break;
      }
      else if (numAudioFrames == 0)
      {
        Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        // falls through to repeating from the top of for(;;) and polls again without returning an empty AudioData - this is intended
      }
      else // we have some audio frame
      {
        // Deinterleave the frames for the wakeupAPI
        waitBuffer.insert(waitBuffer.end(), audioFramesInt.begin(),
                          audioFramesInt.begin() + (numAudioFrames * numChannels));

        const size_t maxWaitSize = wakeupAPIBufferSize * numChannels * 2;
        if (waitBuffer.size() > maxWaitSize)
          waitBuffer.erase(waitBuffer.begin(), waitBuffer.end() - maxWaitSize);

        if (waitBuffer.size() > (wakeupAPIBufferSize * numChannels))
        {
          // deinterleave from waitBuffer
          for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++)
          {
            inputAudioBuffers[0][bufferInd] = waitBuffer[bufferInd * numChannels + 0];
            inputAudioBuffers[1][bufferInd] = waitBuffer[bufferInd * numChannels + 1];
            inputAudioBuffers[2][bufferInd] = waitBuffer[bufferInd * numChannels + 2];
            inputAudioBuffers[3][bufferInd] = waitBuffer[bufferInd * numChannels + 4];
          }

          Duerwen_wakeup_three_write_data(wakeupHandle, inputAudioBuffers[0].data(), inputAudioBuffers[1].data(),
                                          inputAudioBuffers[2].data(), inputAudioBuffers[3].data(),
                                          outputAudioBuffers[0].data(), outputAudioBuffers[1].data(),
                                          outputAudioBuffers[2].data(), outputAudioBuffers[3].data());

          // interleave the output back into audioFrameInt
          for (size_t bufferInd = 0; bufferInd < wakeupAPIBufferSize; bufferInd++)
          {
            audioFramesInt[bufferInd * 3 + 0] = outputAudioBuffers[0][bufferInd];
            audioFramesInt[bufferInd * 3 + 1] = outputAudioBuffers[1][bufferInd];
            audioFramesInt[bufferInd * 3 + 2] = outputAudioBuffers[2][bufferInd];
          }

          // erase the waitBuffer data we just processed
          waitBuffer.erase(waitBuffer.begin(), waitBuffer.begin() + (wakeupAPIBufferSize * numChannels));

          // finally convert the signed audioFramesInt samples to float samples
          unsigned firstSample = 0; // audioData.samples.size();
          unsigned samplesSize = wakeupAPIBufferSize * audioData.channels;
          audioData.samples.resize(firstSample + samplesSize);
          for (unsigned sampleIndex = firstSample; sampleIndex < (firstSample + samplesSize); sampleIndex++)
            audioData.samples[sampleIndex] = static_cast<float>(audioFramesInt[sampleIndex]) / INT16_MAX;

          break;
        }
        else
        {
          // If we read nonzero frames but they weren't enough to complete a
          // wakeupAPI chunk (the `if` above didn't run), we'd otherwise loop
          // straight back to snd_pcm_avail with no sleep at all. Add the same
          // short throttle here so partial-chunk reads can't spin tightly.
          Thread::sleep(10); // TODO: size to alsaPeriodFrames once queried via snd_pcm_hw_params_get_period_size
        }
      }
    }
  }

  audioTimestamp = Time::getCurrentSystemTime();
#else // not TARGET_ROBOT
  audioData.samples.clear();
  Thread::sleep(30);
  audioTimestamp = Time::getCurrentSystemTime();
#endif
}
