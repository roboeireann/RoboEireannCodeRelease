/**
 * @file: WhistleDetectionRE2023.cpp
 *
 * This is the new RoboEireann whistle detector designed for the
 * BH2021 framework.
 *
 * @author: Shauna Recto
 */

#include "WhistleDetectorRE2023.h"

#include "Tools/Debugging/Annotation.h"
#include "Platform/Thread.h"                            // Used for syncing
#include "Platform/File.h"                              // Used for file locations

#include "Tools/TextLogging.h"


DECL_TLOGGER(tlogger, "WhistleDetectorRE2023", TextLogging::INFO);


MAKE_MODULE(WhistleDetectorRE2023, modeling);

static DECLARE_SYNC;

WhistleDetectorRE2023::WhistleDetectorRE2023()
{
  /* Obtain location for Whistle Detector model */
  const std::string baseDir = std::string(File::getBHDir()) + "/Config/NeuralNets/WhistleDetectorRE2023/";
  whistleDetModel = std::make_unique<NeuralNetwork::Model>(baseDir + whistleDetName);

  /* Memory Allocation for FFT */
  spectrumSize = static_cast<int>((fftSize / 2) + 1);
  samples = fftwf_alloc_real(fftSize);
  spectrum = fftwf_alloc_complex(spectrumSize);

  SYNC;

  /* Compile NN */
  whistleDetCompNN.compile(*whistleDetModel);

  /* Implementing FFTW plans */
  fft = fftwf_plan_dft_r2c_1d(fftSize, samples, spectrum, FFTW_MEASURE);

  /* Memory allocation for averaging */
  whistleConfidences.resize(averagingLength);

  /* Initialize samples buffer are all set to 0 */
  std::memset(samples, 0, sizeof(float) * fftSize);
  std::fill(whistleConfidences.begin(), whistleConfidences.end(), 0);

  /* Implement Hanning Window */
  hannWindow.resize(fftSize);
  for (int i = 0; i < fftSize; i++)
    hannWindow[i] = static_cast<float>(std::pow(std::sin(pi * i / fftSize), 2));

  /* Resize magSpectrums according specifications */
  magSpectrums.reserve(whistleDetCompNN.input(0).dims(0));

  /* Make initial empty spectrums which will be dropped later on */
  for (std::vector<float> &magSpect : magSpectrums)
  {
    magSpect.resize(spectrumSize);
    std::fill(magSpect.begin(), magSpect.end(), 0);
  }

  /* Set our leftOverBuffer */
  leftOverBuffer.reserve(1);
  leftOverBuffer.push_front(0);
}

WhistleDetectorRE2023::~WhistleDetectorRE2023()
{
  SYNC;
  fftwf_destroy_plan(fft);
  fftwf_free(spectrum);
  fftwf_free(samples);
}

void WhistleDetectorRE2023::update(Whistle &theWhistle)
{
  /* Plot Declarations */
  DECLARE_PLOT("module:WhistleDetectorRE2023:samples0");
  DECLARE_PLOT("module:WhistleDetectorRE2023:samples1");
  DECLARE_PLOT("module:WhistleDetectorRE2023:samples2");
  DECLARE_PLOT("module:WhistleDetectorRE2023:samples3");
  DECLARE_PLOT("module:WhistleDetectorRE2023:audioPower");
  DECLARE_PLOT("module:WhistleDetectorRE2023:monoBuffer");
  DECLARE_PLOT("module:WhistleDetectorRE2023:whistleConfidence");
  DECLARE_PLOT("module:WhistleDetectorRE2023:bestWhistleConfidence");
  DECLARE_PLOT("module:WhistleDetectorRE2023:whistleConfidenceThresh");
  DECLARE_PLOT("module:WhistleDetectorRE2023:upperWhistleConfidenceThresh");

  /* Checks:
   * 1. Whether the Audio Provider is currently feeding samples.
   * 2. The robot is currently in the SET or PLAYING state.
   */
  bool validAudio = (!theAudioData.samples.empty() && (theAudioData.channels >= 1));
  bool detectWhistles = (theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING);

  /* If not receiving audio input */
  if (!validAudio)
  {
    /* Do nothing, leave Whistle representation with details for prev whistle detected */
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection this time
  }
  /* go for whistle detection */
  else
  {
    /* Adapt number of channels to audio data; we assume this only changes at startup */
    channelBuffers.resize(theAudioData.channels);
    for (auto &buffer : channelBuffers)
      buffer.reserve(bufferSize);

    processAudio(theWhistle, detectWhistles);
  }
}

/* TODO: Check below if true.
 * I am assuming we no longer need to filter noise from the audio itself considering
 * the ML model utilized noisy samples in its dataset. Therefore this function should
 * only run when we need it to. - Shauna
 *
 * Processing of this audio is a 2 stage process.
 *
 * [Stage 1 - Buffer Filling]
 *  We must obtain the audio from the AudioProvider for further processing.
 *
 * [Stage 2 - Downmixing]
 *  Since this ML model uses a mono stream of data, we must downmix channels to
 *  a single channel.
 */
void WhistleDetectorRE2023::processAudio(Whistle &theWhistle, bool detectWhistles)
{
  /* Couple of things to consider
   * 1. Samples from AudioProvider are interleaved meaning that each sample per channel
   *    increments only after the previous channel gets their audio sample.
   *    [Example] -> [C1N0][C2N0][C3N0][C4N0][C1N1][C2N1][C3N1]...
   * 2. stepSize helps get only samples for a specific channel, skipping other channels' samples.
   * 3. Since we may have previously recieved but unused samples, we can now push them first to
   *    the monoBuffer (remember to read ring buffer correctly).
   */
  const size_t stepSize = theAudioData.channels;
  monoBuffer.reserve((theAudioData.samples.size() / stepSize) + leftOverBuffer.size());

  /* Place samples from leftOverBuffer to monoBuffer */
  for (size_t sample = 0; sample < leftOverBuffer.size(); sample++)
    monoBuffer.push_front(leftOverBuffer.from_back(sample));

  /* Copy from interleaved audioData to separate ring buffers, one per channel
   * (only the most recent bufferSize samples will be available in the ring buffer
   * after this)
   */
  for (size_t i = 0; i < theAudioData.samples.size(); i += stepSize)
  {
    /* Total sum of samples */
    float monoSum = 0;

    /* For each channel */
    for (size_t channel = 0; channel < theAudioData.channels; ++channel)
    {
      /* Obtain desired audio sample and perform clamping checks before
       * pushing to buffers.
       */
      const float audioSample = clampSample(theAudioData.samples[i + channel], 2.f);
      channelBuffers[channel].push_front(audioSample);

      /* add samples from all channels into one mono sample */
      monoSum += audioSample;

      /* Plot samples based on current sample set */
      switch (channel)
      {
      case 0:
        PLOT("module:WhistleDetectorRE2023:samples0", channelBuffers[channel].back());
        break;
      case 1:
        PLOT("module:WhistleDetectorRE2023:samples1", channelBuffers[channel].back());
        break;
      case 2:
        PLOT("module:WhistleDetectorRE2023:samples2", channelBuffers[channel].back());
        break;
      case 3:
        PLOT("module:WhistleDetectorRE2023:samples3", channelBuffers[channel].back());
        break;
      default:
        break;
      }
    }

    /* normalize the mono sample level and add to the mono mix buffer */
    monoBuffer.push_front(clampSample(monoSum / theAudioData.channels * sampleGain, 1.f));

    /* Plot monoBuffer Data
     * TODO: Check whether this is the right way for indexing through monoBuffer.
     */
    PLOT("module:WhistleDetectorRE2023:monoBuffer", monoBuffer[i]);
  }

  /* We process our buffer as multiple FFT windows.
   * We determine the whistleConfidence from all FFT windows passed
   * into our NN which produces a single float value */
  float whistleConfidence = compareAudio();

  /* Set the detection info */
  if (detectWhistles)
  {
    theWhistle.confidenceOfLastWhistleDetection = whistleConfidence;
    theWhistle.channelsUsedForWhistleDetection = static_cast<unsigned char>(channelBuffers.size());
    if (whistleConfidence > 0.f)
    {
      if (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) > detTimeoutMs)
        theWhistle.lastTimeWhistleDetected = theFrameInfo.time;
    }
  }
  else
  {
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection enabled at this time
  }
}

/* The purpose of this is analagous to processWindowMonoMix() from the 2017, however,
 * the use of the ML model will be utilized and no filtering would be done.
 *
 * This function also has 2 stages:
 * [FFTW Computation]
 *  The FFT of the selected samples will be calculated and then passed for comparison.
 *
 * [NN Computation]
 *  The resultant spectrum will be compared.
 */
float WhistleDetectorRE2023::compareAudio()
{
  /* Process the window of data starting at oldest.
   * NOTE: Ringbuffers are indexed from newest to oldest so we need
   * to reverse this when populating the fft samples.
   * Done with .from_back()
   */

  /* Gets the number of FFTs to compute.
   * fftOffset sets the starting point of the monoBuffer to read from.
   * numFFTs is capped to 15, with the excess left for next cycle.
   */
  int numFFTs = std::min(15, static_cast<int>((monoBuffer.size() / fftStep)));
  int fftOffset = 0;

  TLOGD(tlogger, "Total FFTs to compute {}", numFFTs);

  /* For all the available FFT from buffer */
  for (int i = 0; i < numFFTs; i++)
  {
    /* Debugging value to determine if spectrum is outputting legal values */
    float audioPower = 0.f;

    /* Push new FFT vector into RingBuffer */
    magSpectrums.push_front(std::vector<float>());
    magSpectrums.front().resize(spectrumSize);

    /* Get our buffer samples for the FFT, with gain scaling */
    for (int sampleIndex = 0; sampleIndex < fftSize; sampleIndex++)
      samples[sampleIndex] = monoBuffer.from_back(fftOffset + sampleIndex) * hannWindow[sampleIndex];

    /* Perform the FFT on current set of samples */
    fftwf_execute(fft);
    for (int j = 0; j < spectrumSize; j++)
    {
      /* calculate the magnitude spectrum with usual FFT normalization */
      magSpectrums.front()[j] =
          clampSample((spectrum[j][0] * spectrum[j][0] + spectrum[j][1] * spectrum[j][0]) / fftSize, 1.f);
      audioPower += magSpectrums.front()[j];
    }

    // TLOGD("Audio Power is: {} ", audioPower);
    PLOT("module:WhistleDetectorRE2023:audioPower", audioPower);

    /* Use the next set of samples */
    fftOffset += fftStep;
  }

  /* Store excess samples into leftOverBuffer.
   * Make assertions to determine if there are samples to hold for next cycle.
   * Then determine the number of samples left over.
   */
  size_t leftOverSize = monoBuffer.size() + fftStep - (numFFTs * fftStep);

  TLOGD(tlogger, "There are {} leftover samples ({} - {})", leftOverSize, monoBuffer.size() + fftStep,
        (numFFTs * fftStep));
  ASSERT(leftOverSize > 0);

  leftOverBuffer.reserve(leftOverSize);

  /* Position the pointer in RingBuffer of last sample to be used.
   * As the previous loop ends with the last offset + an unused step,
   * revert said change.
   */
  size_t leftOverOffset = fftOffset - fftStep;

  /* Iterate over the remaining samples to be put into the leftOverBuffer. */
  for (size_t sample = 0; sample < leftOverSize; sample++)
    leftOverBuffer.push_front(monoBuffer.from_back(leftOverOffset + sample));

  /* Assertions of the NN model */
  // TLOGD(tlogger, "Current number of inputs: {}", whistleDetCompNN.numOfInputs());
  // TLOGD(tlogger, "Current number of outputs: {}", whistleDetCompNN.numOfOutputs());
  // TLOGD(tlogger, "Current input dimensions: {} x {} x {}", whistleDetCompNN.input(0).dims(0),
  // whistleDetCompNN.input(0).dims(1), whistleDetCompNN.input(0).dims(2)); TLOGD(tlogger, "Current output dimensions:
  // {}", whistleDetCompNN.output(0).dims(0));

  ASSERT(whistleDetCompNN.numOfInputs() == 1);
  ASSERT(whistleDetCompNN.numOfOutputs() == 1);

  /* This should fit the input dimension of 15x129x1 */
  ASSERT(whistleDetCompNN.input(0).dims(0) == 15);
  ASSERT(whistleDetCompNN.input(0).dims(1) == 129);
  ASSERT(whistleDetCompNN.input(0).dims(2) == 1);

  /* We only want 1 output */
  ASSERT(whistleDetCompNN.output(0).dims(0) == 2);

  /* Passing FFTs into NN */
  float *fftDest = whistleDetCompNN.input(0).data();

  /* TODO: Determine if the properties of the magSpectrums RingBuffer affect the results of WD */
  // for(const std::vector<float> &magSpect : magSpectrums)
  // {
  //     memcpy(fftDest, magSpect.data(), magSpect.size() * sizeof(float));
  //     fftDest += magSpect.size() * sizeof(float);
  // }

  /* Adapted for loop to gurantee taking from the back of RingBuffer */
  for (size_t magSpectInd = 0; magSpectInd < magSpectrums.size(); magSpectInd++)
  {
    memcpy(fftDest, magSpectrums.from_back(magSpectInd).data(), spectrumSize * sizeof(float));
    fftDest += spectrumSize * sizeof(float);
  }

  /* Comparison of FFT and NN.
   * The whistleBias is used to offset the confidence if
   * it rests outside the -1, 1 range.
   * TODO: Investigate this if any bias is needed in the CNN.
   */
  whistleDetCompNN.apply();
  const float whistleConfidence = whistleDetCompNN.output(0).data()[0] - whistleBias;

  /* Prints values of the whistleConfidence. Debugging. */
  TLOGD(tlogger, "WhistleConfidence Value is {}", whistleConfidence);
  PLOT("module:WhistleDetectorRE2023:whistleConfidence", whistleConfidence);

  /* Smoothening of the confidence.
   * The aim of this is to pick the best confidence value.
   * After testing with the current whistle detector,
   * there are random and sudden spikes, which can be
   * attentuated via averaging smoothing.
   * However, it will output unchanged confidences over
   * upperWhistleConfidenceThresh.
   */
  static int whistConfN = 0;
  whistConfN = whistConfN < averagingLength ? whistConfN++ : 0;
  float averagedWhistleConfidence = 0;

  /* Get all the confidences stored up and average the result*/
  whistleConfidences[whistConfN] = whistleConfidence;
  for (auto &n : whistleConfidences)
    averagedWhistleConfidence += n;

  averagedWhistleConfidence /= averagingLength;

  /* If the values are above the upper threshold and less than 0, do nothing,
   * otherwise use the averaged value.
   */
  float bestWhistleConfidence = ((whistleConfidence > upperWhistleConfidenceThresh) || (whistleConfidence < 0))
                                    ? whistleConfidence
                                    : averagedWhistleConfidence;

  PLOT("module:WhistleDetectorRE2023:bestWhistleConfidence", bestWhistleConfidence);
  PLOT("module:WhistleDetectorRE2023:whistleConfidenceThresh", whistleConfidenceThresh);
  PLOT("module:WhistleDetectorRE2023:upperWhistleConfidenceThresh", upperWhistleConfidenceThresh);

  /* Checks whether the confidence value from the NN */
  if (bestWhistleConfidence > whistleConfidenceThresh)
    ANNOTATION("Whistle", fmt::format("Whistle has been detected with confidence of {}", bestWhistleConfidence));

  return bestWhistleConfidence;
}

float WhistleDetectorRE2023::clampSample(float audioSample, float limit)
{
  if (-limit <= audioSample && audioSample <= limit)
    return audioSample;
  else if (audioSample < -limit)
    return limit;
  else if (audioSample > limit)
    return limit;
  else // nans, infs, etc
    return 0.f;
}
