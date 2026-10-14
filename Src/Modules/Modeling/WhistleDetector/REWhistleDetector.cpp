/**
 * @file: REWhistleDetector.cpp
 *
 * @description This is the RoboEireann whistle detector source ported the
 * BH2025 framework.
 *
 * @author: Shauna Recto
 */

#include "REWhistleDetector.h"

#include "Debugging/Annotation.h"
#include "Debugging/DebugImages.h"
#include "Debugging/Plot.h"

#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"
#include "TextLogging/TextLogging.h"

#include <algorithm>

MAKE_MODULE(REWhistleDetector);

static DECLARE_SYNC;

DECL_TLOGGER(tlogger, "REWhistleDetector", TextLogging::INFO);

REWhistleDetector::REWhistleDetector()
{
  /* Obtain location for Whistle Detector model */
  const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/WhistleDetector/";

  if (model == nullptr)
  {
    model = std::make_unique<NeuralNetworkONNX::Model>(baseDir + whistleDetName);
  }
  else
  {
    TLOGI(tlogger, "Model already loaded...");
  }

  TLOGI(tlogger, "Loading whistle detector model: {}", (baseDir + whistleDetName));

  /* Memory Allocation for FFT */
  spectrumSize = static_cast<int>((fftSize / 2) + 1);
  maxFFTs = ((bufferSize - fftSize) / fftStep) + 1;

  if (samples == nullptr)
    samples = fftwf_alloc_real(fftSize);
  if (spectrum == nullptr)
    spectrum = fftwf_alloc_complex(spectrumSize);

  /* Compile NN */
  whistleDetector.compile(*model);

  ASSERT(whistleDetector.numOfInputs() == 1);
  ASSERT(whistleDetector.numOfOutputs() == 1);
  ASSERT(whistleDetector.input(0).dims(0) == static_cast<unsigned>(maxFFTs));
  ASSERT(whistleDetector.input(0).dims(1) == static_cast<unsigned>(spectrumSize));
  ASSERT(whistleDetector.input(0).dims(2) == 1);
  ASSERT(whistleDetector.output(0).dims(0) == 1);

  SYNC;

  if (fft != nullptr)
  {
    fftwf_destroy_plan(fft);
    fft = nullptr;
  }

  /* Implementing FFTW plans */
  if (fft == nullptr)
    fft = fftwf_plan_dft_r2c_1d(fftSize, samples, spectrum, FFTW_MEASURE);

  /* Initialize samples buffer are all set to 0 */
  std::memset(samples, 0, sizeof(float) * fftSize);

  /* Implement Hanning Window */
  hannWindow.resize(fftSize);
  for (int i = 0; i < fftSize; i++)
    hannWindow[i] = static_cast<float>(std::pow(std::sin(pi * i / fftSize), 2));

  magSpectrums.clear();
  monoBuffer.clear();

  /* Resize magSpectrums according specifications */
  magSpectrums.reserve(whistleDetector.input(0).dims(0));
  monoBuffer.reserve(bufferSize);

  extraInit = false;
}

REWhistleDetector::~REWhistleDetector()
{
  SYNC;

  magSpectrums.clear();
  monoBuffer.clear();
  channelBuffers.clear();

  if (fft != nullptr)
    fftwf_destroy_plan(fft);

  if (spectrum != nullptr)
    fftwf_free(spectrum);

  if (samples != nullptr)
    fftwf_free(samples);
}

void REWhistleDetector::update(Whistle &theWhistle)
{
  DECLARE_PLOT("module:REWhistleDetector:audioBuffer");
  DECLARE_PLOT("module:REWhistleDetector:whistleConfidence");
  DECLARE_PLOT("module:REWhistleDetector:whistleConfidenceThresh");

  bool validAudio = ((!theAudioData.samples.empty()) && (theAudioData.channels >= static_cast<unsigned>(numChannels)));

  /* If not receiving audio input */
  if (!validAudio)
  {
    /* Do nothing, leave Whistle representation with details for prev whistle detected */
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection this time

  } /* go for whistle detection */
  else
  {
    if (!extraInit)
    {
      /* Adapt number of channels to audio data; we assume this only changes at startup */
      channelBuffers.resize(theAudioData.channels);
      for (auto &buffer : channelBuffers)
      {
        buffer.reserve(bufferSize);
      }

      /* Initialize the Noise filter params */
      const float windowLenMs = 1000.0f * fftStep / theAudioData.sampleRate;
      noisefilter.setParams(spectrumSize, static_cast<unsigned>((noiseAvgLenMs) / windowLenMs));
      extraInit = true;
    }

    processAudio(theWhistle, true);
  }
}

void REWhistleDetector::processAudio(Whistle &theWhistle, bool detectWhistle)
{
  /* Couple of things to consider
    * 1. Samples from AudioProvider are interleaved meaning that each sample per channel
    *    increments only after the previous channel gets their audio sample.
    *    [Example] -> [C1N0][C2N0][C3N0][C4N0][C1N1][C2N1][C3N1]...
    * 2. stepSize helps get only samples for a specific channel, skipping other channels' samples.
    * 3. Since we may have previously recieved but unused samples, we can now push them first to
    *    the monoBuffer (remember to read ring buffer correctly).
    */
  const std::size_t stepSize = theAudioData.channels;

  /* Copy from interleaved audioData to separate ring buffers, one per channel
     * (only the most recent bufferSize samples will be available in the ring buffer
     * after this)
     */
  for (std::size_t i = 0; i + stepSize <= theAudioData.samples.size(); i += stepSize)
  {
    /* Total sum of samples */
    float monoSum = 0;

    /* For each channel */
    for (std::size_t channel = 0; channel < static_cast<unsigned>(numChannels); ++channel)
    {
      /* Obtain desired audio sample and perform clamping checks before
             * pushing to buffers.
             */
      const float audioSample = std::clamp(theAudioData.samples[i + channel], -1.f, 1.f);
      channelBuffers[channel].push_front(audioSample);

      /* add samples from all channels into one mono sample */
      monoSum += audioSample;
    }

    /* normalize the mono sample level and add to the mono mix buffer */
    monoBuffer.push_front(std::clamp(monoSum / numChannels * sampleGain, -1.f, 1.f));
    PLOT("module:REWhistleDetector:audioBuffer", monoBuffer[0]);
  }

  /* We process our buffer as multiple FFT windows.
     * We determine the whistleConfidence from all FFT windows passed
     * into our NN which produces a single float value */
  float whistleConfidence = inferenceCheck();

  /* Set the whistleDetection to false at start of the block */
  theWhistle.whistleDetected = false;

  /* Set the detection info */
  if (detectWhistle)
  {
    /* Checks whether the confidence value from the NN */
    if (whistleConfidence > whistleConfidenceThresh)
    {
      theWhistle.whistleDetected = true;
      theWhistle.lastTimeWhistleDetected = theFrameInfo.time;

      theWhistle.confidenceOfLastWhistleDetection = whistleConfidence;
      theWhistle.channelsUsedForWhistleDetection = static_cast<unsigned char>(channelBuffers.size());
    }
  }
  else
  {
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection enabled at this time
  }
}

float REWhistleDetector::inferenceCheck()
{
  /* Process the window of data starting at oldest.
     * NOTE: Ringbuffers are indexed from newest to oldest so we need
     * to reverse this when populating the fft samples.
     * Done with .from_back()
     */

  /* Gets the number of FFTs to compute.
     * fftOffset sets the starting point of the monoBuffer to read from.
     * numFFTs is capped to 18, with the excess left for next cycle.
     */
  int numFFTs = std::min(maxFFTs, static_cast<int>((monoBuffer.size() - fftSize) / fftStep) + 1);
  if (static_cast<int>(monoBuffer.size()) < fftSize)
    numFFTs = 0; // not enough samples yet for even one full window

  int fftOffset = 0;
  /* For all the available FFT from buffer */
  for (int i = 0; i < numFFTs; i++)
  {
    /* Push new FFT vector into RingBuffer */
    magSpectrums.push_front(std::vector<float>());
    magSpectrums.front().resize(spectrumSize);

    /* Get our buffer samples for the FFT, with gain scaling */
    for (int sampleIndex = 0; sampleIndex < fftSize; sampleIndex++)
    {
      samples[sampleIndex] = monoBuffer.from_back(fftOffset + sampleIndex) * hannWindow[sampleIndex];
      bool hasNaN = false;
      if (std::isnan(samples[sampleIndex])) hasNaN = true;
      if (hasNaN)
        TLOGW(tlogger, "NaN value detected...");
    }

    /* Perform the FFT on current set of samples */
    fftwf_execute(fft);
    for (int j = 0; j < spectrumSize; j++)
    {
      /* calculate the magnitude spectrum with usual FFT normalization */
      magSpectrums.front()[j] =
          std::clamp(std::sqrt(spectrum[j][0] * spectrum[j][0] + spectrum[j][1] * spectrum[j][1]), 0.f, 1.f);
    }

    /* Update our filter profile */
    noisefilter.update(magSpectrums.front());

    /* Subtract noise profile from magSpectrums */
    for (int j = 0; j < spectrumSize; j++)
    {
      /* calculate the magnitude spectrum with usual FFT normalization */
      magSpectrums.front()[j] = std::max(magSpectrums.front()[j] - noisefilter.spectProfile[j], 0.f);
    }

    /* Use the next set of samples */
    fftOffset += fftStep;
  }

  TLOGD(tlogger, "magSpectrums.size()={} maxFFTs={}", magSpectrums.size(), maxFFTs);
  /* In the event we don't have enough FFTs for an inference check */
  if (static_cast<int>(magSpectrums.size()) < maxFFTs)
    return 0.f;

  /* Passing FFTs into NN */
  float *fftDest = whistleDetector.input(0).data();

  /* Adapted for loop to gurantee taking from the back of RingBuffer */
  for (std::size_t magSpectInd = 0; magSpectInd < magSpectrums.size(); magSpectInd++)
  {
    memcpy(fftDest, magSpectrums.from_back(magSpectInd).data(), spectrumSize * sizeof(float));
    fftDest += spectrumSize;
  }

  /* Inference check */
  whistleDetector.apply();
  const float whistleConfidence = whistleDetector.output(0).data()[0];
  //TLOGI(tlogger, "Confidence Value: {}", whistleConfidence);
  /* Prints values of the whistleConfidence. Debugging. */
  PLOT("module:REWhistleDetector:whistleConfidence", whistleConfidence);
  PLOT("module:REWhistleDetector:whistleConfidenceThresh", whistleConfidenceThresh);

  return whistleConfidence;
}
void REWhistleDetector::NoiseFilter::setParams(unsigned spectSize, unsigned sampleAvg)
{
  spectProfile.resize(spectSize, 0);
  numSample = 0;
  maxNumSample = sampleAvg;
}

void REWhistleDetector::NoiseFilter::update(const std::vector<float> &inputSpect)
{
  if (numSample++ == 0)
  {
    for (size_t binInd = 0; binInd < spectProfile.size(); binInd++)
      spectProfile[binInd] = inputSpect[binInd];
  }
  else
  {
    float smoothAdjust = 2.0f / (numSample + 1);

    for (size_t binInd = 0; binInd < spectProfile.size(); binInd++)
      spectProfile[binInd] = (smoothAdjust * inputSpect[binInd]) + ((1 - smoothAdjust) * spectProfile[binInd]);
  }

  if (numSample > maxNumSample)
    numSample = maxNumSample;
}
