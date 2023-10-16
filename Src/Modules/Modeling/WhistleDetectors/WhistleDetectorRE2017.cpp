/**
 * @file: WhistleDetectorRE2017.cpp
 *
 * This a version of the RoboEireann 2017 Whistle Detector adapted to the
 * BH2021 framework
 *
 * The detector is pretty simple but not selective enough at distinguishing
 * whistle sounds from other load broadband noises
 *
 * - constantly evaluate the long term moving average of the sound power which
 *   we assume is primarily "background noise", e.g. fan noise, robot movement
 *   noises, audience sounds etc. Remove this long term average using
 *   spectral subtraction
 *
 * - calculate a short term moving average of sound power in a selected range
 *   of frequencies that should primarily contain whistle sounds. If this
 *   short term average exceeds a threshold, consider this to be a candidate
 *   whistle
 *
 * @author: Rudi Villing
 * @author: Shauna Recto
 */

#include "WhistleDetectorRE2017.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/Math/Constants.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include "fmt/core.h"

#include <cmath>


DECL_TLOGGER(tlogger, "WhistleDetectorRE2017", TextLogging::INFO);

static DECLARE_SYNC; // TODO - is this really needed?

MAKE_MODULE(WhistleDetectorRE2017, modeling);


void WhistleDetectorRE2017::Smoothed::update(float* input)
{
  // smoothing using exponential averaging
  if (n == 0)
  {
    for (size_t i = 0; i < smoothed.size(); i++)
      smoothed[i] = input[i];
    n++;
  }
  else
  {
    if (n < nAvg) // will max out at nAvg
      n++;
    else
      n = nAvg;

    // gradually adjust the smoothing factor as we add windows until we
    // reach the target number for averaging
    float alpha = 2.0f / (n + 1);


    // TLOGD(tlogger, "smoothed: n={}, alpha={}, data=", n, alpha);

    // add the input and smooth
    for (size_t i = 0; i < smoothed.size(); i++)
    {
      // exponentially averaged (background) noise power (over long averaging period)
      smoothed[i] = alpha * input[i] + (1 - alpha) * smoothed[i];

      // if (i < 6)
      //   TLOGD(tlogger, "smoothed {}", smoothed[i]);
    }
  }
}


WhistleDetectorRE2017::WhistleDetectorRE2017()
{
  spectrumSize = (fftSize / 2) + 1;

  // Allocate memory for FFTW plans
  samples = fftwf_alloc_real(fftSize);
  spectrum = fftwf_alloc_complex(spectrumSize);

  // Create FFTW plans
  SYNC; // to prevent deallocating resources that are only partially allocated?

  fft = fftwf_plan_dft_r2c_1d(fftSize, samples, spectrum, FFTW_MEASURE);

  // ensure input FT buffer is initially zero'd
  std::memset(samples, 0, sizeof(float) * fftSize);
  std::memset(spectrum, 0, sizeof(fftwf_complex) * spectrumSize); // TODO: is this needed?

  monoNoiseMagnitude.resize(spectrumSize, 0.f);
  monoSignalMagnitude.resize(spectrumSize, 0.f);
  monoSubtractedMagnitude.resize(spectrumSize, 0.f);

  // for (int i=0; i<spectrumSize; i++)
  // {
  //   if (monoNoiseMagnitude.smoothed[i] != 0.f)
  //     FAIL("monoNoise not init'd zero");
  //   if (monoSignalMagnitude.smoothed[i] != 0.f)
  //     FAIL("monoSignal not init'd zero");
  // }

  initExtraComplete = false;
}

void WhistleDetectorRE2017::initExtra()
{
  // these values depend on representations only available when executing first update cycle
  float sampleRate = theAudioData.sampleRate;
  float binWidthHz = sampleRate / fftSize;

  ASSERT(minFreqHz >= 0);
  maxFreqHz = std::min(maxFreqHz, sampleRate / 2);

  minFFTBin = int(minFreqHz / binWidthHz);
  maxFFTBin = int(maxFreqHz / binWidthHz);

  // LOGI(logger, "init: fftBinWidthHz {:.2f}, using frequencies {}-{} (bins {}-{})",
  //         binWidthHz, p.minFreqHz, p.maxFreqHz, minFFTBin, maxFFTBin);

  // hann windows

  hannWindow.resize(fftSize);
  for (int i=0; i<fftSize; i++)
    hannWindow[i] = (float) std::pow(std::sin(pi * i / fftSize), 2);

  // initialize exponential averagers

  float windowMs = 1000.0f * fftAdvance / sampleRate;

  monoNoiseMagnitude.setNumAverage(unsigned(noiseAvgMs / windowMs));
  monoSignalMagnitude.setNumAverage(unsigned(audioAvgMs / windowMs));
  monoSubtractedMagnitude.setNumAverage(unsigned(audioAvgMs / windowMs));

  TLOGI(tlogger, "monoNoiseMagnitude nAvg = {}, monoSignalMagnitude nAvg = {}", 
        monoNoiseMagnitude.nAvg, monoSignalMagnitude.nAvg);

  // numWindowsRequired = std::max(monoNoiseMagnitude.nAvg, monoSignalMagnitude.nAvg);

  monoBuffer.reserve(bufferSize);

  // LOGI(logger, "init: windowMs {:.1f}, noise N {} alpha {:.4f}, audio N {} alpha {:.4f}",
  //         binWidthHz, noiseAvgCount, alphaNoise, audioAvgCount, alphaAudio);

  initExtraComplete = true;
}

WhistleDetectorRE2017::~WhistleDetectorRE2017()
{
  SYNC;
  fftwf_destroy_plan(fft);
  fftwf_free(spectrum);
  fftwf_free(samples);
}

void WhistleDetectorRE2017::update(Whistle &theWhistle)
{
  if (!initExtraComplete)
  {
    initExtra();
  }

  // Plot Declarations
  DECLARE_PLOT("module:WhistleDetectorRE2017:samples0");
  DECLARE_PLOT("module:WhistleDetectorRE2017:samples1");
  DECLARE_PLOT("module:WhistleDetectorRE2017:samples2");
  DECLARE_PLOT("module:WhistleDetectorRE2017:samples3");
  DECLARE_PLOT("module:WhistleDetectorRE2017:monoBuffer");
  DECLARE_PLOT("module:WhistleDetectorRE2017:maxConfidence");

  DECLARE_PLOT("module:WhistleDetectorRE2017:whistleConfidence");
  DECLARE_PLOT("module:WhistleDetectorRE2017:whistleDetected");

  DECLARE_PLOT("module:WhistleDetectorRE2017:whistleBandPower");
  // temporary
  DECLARE_PLOT("module:WhistleDetectorRE2017:sumMagnitudeSpectrum");
  DECLARE_PLOT("module:WhistleDetectorRE2017:sumSubtractedSpectrum");
  DECLARE_PLOT("module:WhistleDetectorRE2017:sumNoiseMagSmoothed");
  DECLARE_PLOT("module:WhistleDetectorRE2017:sumMonoSignalMagSmoothed");
  DECLARE_PLOT("module:WhistleDetectorRE2017:sumMonoSubtractedMagSmoothed");

  DECLARE_PLOT("module:WhistleDetectorRE2017:sufficientSamples");

  // Testing Plot Declarations
  DECLARE_PLOT("module:WhistleDetectorRE2017:debug0");
  DECLARE_PLOT("module:WhistleDetectorRE2017:debug1");

  // Remember if sound was playing during this team communication cycle
  if (SystemCall::soundIsPlaying())
    framesSinceOwnSoundPlaying = 0;
  else if (++framesSinceOwnSoundPlaying > 100)
    framesSinceOwnSoundPlaying = 100;

  // AudioDataProvider decides which state to provide audio and
  // we can additionally decide whether we want to process provided audio
  bool validAudio = (!theAudioData.samples.empty() && (theAudioData.channels >= 1) && (framesSinceOwnSoundPlaying > 1));
  bool detectWhistles = (theGameInfo.state == STATE_SET || theGameInfo.state == STATE_PLAYING);

  // only run whistle detection if enabled
  if (!validAudio)
    // do nothing, leave Whistle representation with details for prev whistle detected
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection this time
  else // go for whistle detection
  {
    // Adapt number of channels to audio data - we assume this only changes at startup
    channelBuffers.resize(theAudioData.channels);
    for (auto& buffer : channelBuffers) 
    {
      buffer.reserve(bufferSize);
    }
      
      if (!(theMotionInfo.isKicking())) {
        processAudio(theWhistle, detectWhistles);
    }
  }
}

void WhistleDetectorRE2017::processAudio(Whistle &theWhistle, bool detectWhistles)
{
  // even if detectWhistles indicates that we don't want to detect whistles 
  // in the current state, we still run the audio processing since
  // part of our processing depends on long term noise estimates and it is
  // better to have that up and running all the time

  // check number of samples provided by AudioData vs what we're prepared to handle
  // every 16.7ms on average (the cognition cycle rate)
  const size_t stepSize = theAudioData.channels;

  size_t numSampleFrames = theAudioData.samples.size() / stepSize;
  if (numSampleFrames > bufferSize)
    ANNOTATION("Whistle", fmt::format("discarding {} sampleFrames", numSampleFrames - bufferSize));

  // copy from interleaved audioData to separate ring buffers, one per channel
  // (only the most recent bufferSize samples will be available in the ring buffer
  // after this)
  for (size_t i=0; i < theAudioData.samples.size(); i += stepSize)
  {
    float monoSum = 0;
    for (size_t channel = 0; channel < theAudioData.channels; ++channel)
    {
      const float audioSample = clampLegal(theAudioData.samples[i + channel]);

      channelBuffers[channel].push_front(audioSample);

      // add samples from all channels into one mono sample
      monoSum += audioSample;

      // Plot samples based on current sample set
      switch (channel)
      {
      case 0: PLOT("module:WhistleDetectorRE2017:samples0", channelBuffers[channel].back()); break;
      case 1: PLOT("module:WhistleDetectorRE2017:samples1", channelBuffers[channel].back()); break;
      case 2: PLOT("module:WhistleDetectorRE2017:samples2", channelBuffers[channel].back()); break;
      case 3: PLOT("module:WhistleDetectorRE2017:samples3", channelBuffers[channel].back()); break;
      default: break;
      }
    }

    // normalize the mono sample level and add to the mono mix buffer
    monoBuffer.push_front(monoSum / theAudioData.channels);

    // Plot monoBuffer Data
    PLOT("module:WhistleDetectorRE2017:monoBuffer", monoBuffer[i]);
  }

  // We process our buffer as multiple FFT windows
  // Determine the maxConfidence from all FFT windows and supply this as
  // our overall confidence. (The underlying detection mechanism already
  // incorporates smoothing/filtering so we don't filter the detections
  // any further here)

  float maxConfidence = 0.f;
  for (size_t i = 0; i < (channelBuffers[0].size() - fftSize); i += fftAdvance)
    maxConfidence = std::max(processWindowMonoMix(i), maxConfidence);

  // Adds the current value of maxConfidence to the current game plot
  PLOT("module:WhistleDetectorRE2017:maxConfidence", maxConfidence);
  TLOGD(tlogger, "maxConfidence Final = {}\n", maxConfidence);

  // set the detection info
  if (detectWhistles)
  {
    theWhistle.confidenceOfLastWhistleDetection = maxConfidence;
    theWhistle.channelsUsedForWhistleDetection = static_cast<unsigned char>(channelBuffers.size());
    if (maxConfidence > 0.f)
    {
      if (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) > minAnnotationIntervalMs)
        ANNOTATION("WhistleDetectorRE2017", fmt::format("whistle detected with confidence {:.2f}", maxConfidence));
      theWhistle.lastTimeWhistleDetected = theFrameInfo.time;
    }
  }
  else
    theWhistle.channelsUsedForWhistleDetection = 0; // no detection enabled at this time
}

float WhistleDetectorRE2017::processWindowMonoMix(size_t start)
{
  // process the window of data starting at oldest
  // NOTE that ringbuffers are indexed from newest to oldest so we need
  // to reverse this when populating the fft samples

  // copy one window of data to the fft buffer and multiply by the window function
  // (to reduce spectral leakage)
  for (int i = 0; i < fftSize; i++)
  {
    samples[i] = monoBuffer.from_back(start+i) * hannWindow[i];
  }

  // FFT the data to produce a complex spectrum
  fftwf_execute(fft);


  // calculate the (real) magnitude spectrum
  float magnitudeSpectrum[spectrumSize];

  for (int i = 0; i < spectrumSize; i++)
  {
    // calculate the magnitude spectrum with usual FFT normalization
    magnitudeSpectrum[i] = sqrtf(spectrum[i][0] * spectrum[i][0] + spectrum[i][1] * spectrum[i][1]) / fftSize;
    TLOGD(tlogger, "magSpect = {}", magnitudeSpectrum[i]);
    TLOGD(tlogger, "Spect = {}", spectrum[i][0]);
  }

  // LOGI("monoSignalMagnitude\n");
  monoSignalMagnitude.update(magnitudeSpectrum);

  bool sufficientSamplesForDetection = monoNoiseMagnitude.n >= std::max(monoNoiseMagnitude.nAvg/2, monoSignalMagnitude.nAvg);

  // do spectral subtraction to get spectrum of the where the signal differs
  // from the long term ambient noise

  float subtractedSpectrum[spectrumSize];

  for (int i = 0; i < spectrumSize; i++)
  {
    // have we added sufficient samples to stabilise the background noise estimate?
    if (sufficientSamplesForDetection)
      subtractedSpectrum[i] = std::max(magnitudeSpectrum[i] - monoNoiseMagnitude.smoothed[i], 0.f);
    else
      subtractedSpectrum[i] = 0.f;
  }

  // update the smoothed signal (with noise subtracted) estimate
  // LOGI("monoSubtractedMagnitude\n");
  monoSubtractedMagnitude.update(subtractedSpectrum);


  float sumMagnitudeSpectrum = 0.f;
  float sumSubtractedSpectrum = 0.f;
  float sumMonoSignalMagSmoothed = 0.f;
  float sumMonoSubtractedMagSmoothed = 0.f;
  for (int i = 0; i < spectrumSize; i++)
  {
    sumMagnitudeSpectrum += magnitudeSpectrum[i];
    sumSubtractedSpectrum += subtractedSpectrum[i];
    sumMonoSignalMagSmoothed += monoSignalMagnitude.smoothed[i];
    sumMonoSubtractedMagSmoothed += monoSubtractedMagnitude.smoothed[i];
  }
  // avgMagnitudeSpectrum /= spectrumSize;
  // avgSubtractedSpectrum /= spectrumSize;

  // all of the plots in this function produce one output value per fftAdvance samples

  PLOT("module:WhistleDetectorRE2017:sumMagnitudeSpectrum", sumMagnitudeSpectrum);
  PLOT("module:WhistleDetectorRE2017:sumSubtractedSpectrum", sumSubtractedSpectrum);

  PLOT("module:WhistleDetectorRE2017:sumMonoSignalMagSmoothed", sumMonoSignalMagSmoothed);
  PLOT("module:WhistleDetectorRE2017:sumMonoSubtractedMagSmoothed", sumMonoSubtractedMagSmoothed);


  TLOGD(tlogger, "monoSignalMagnitude After = {}, {}", monoSignalMagnitude.n, monoSignalMagnitude.nAvg);

  // Spectrogram Definitions
  // COMPLEX_DRAWING("module:WhistleDetectorRE2017:magnitudeSpectrum")
  // {
  //   // Initialize the spectrogram (all pixels)
  //   std::memset(magSpectrogram[0], 128, sizeof(PixelTypes::Edge2Pixel) * magSpectrogram.width * magSpectrogram.height);

  //   // Iterate through each value from the spectrum
  //   for (int i = 0; i < spectrumSize; i++)
  //   {
  //     if (magnitudeSpectrum[i] > 0)
  //     {
  //       const PixelTypes::Edge2Pixel pixel(static_cast<char>(std::min((255 * amplitude), 255)));
  //     }

  //     magSpectrogram[0][i] = pixel;
  //   }
  // }

  // COMPLEX_DRAWING("module:WhistleDetectorRE2017:subtractedSpectrum")
  // {

  // }

  // check for a potential whistle sound if we have enough data
  bool whistleDetected = false;
  float whistleConfidence = 0.f;
  
  if (sufficientSamplesForDetection)
  {
    // sum the smoothed (noise-subtracted) signal power spectral densities across 
    // the FFT bins corresponding to the whistle band
    float whistleBandPower = 0.0;

    for (int i = minFFTBin; i <= maxFFTBin; i++)
    {
      // whistleBandPower += (monoSignalMagnitude.smoothed[i] * monoSignalMagnitude.smoothed[i]); // square the magnitude to get power
      whistleBandPower += monoSubtractedMagnitude.smoothed[i]; // FIXME: this is not power
      // TLOGD(tlogger, "avgWhistleBandPower adding = {}", avgWhistleBandPower);
      TLOGD(tlogger, "smoothed Noise = {}", monoNoiseMagnitude.smoothed[i]);
    }

    // normalize this to same sort of numeric range as the original signal power
    // (even though it is only a fraction of it), so that we can change the number
    // of fft bins without having to completely retune the threshold

    whistleBandPower = whistleBandPower * spectrumSize / (maxFFTBin - minFFTBin + 1);

    // Plot the avgWhistleBandPower in SimRobot
    PLOT("module:WhistleDetectorRE2017:whistleBandPower", whistleBandPower);

    // TLOGD(tlogger, "avgWhistleBandPower Final = {}", avgWhistleBandPower);

    if (whistleBandPower > detectionPower)
    {
      whistleDetected = true;
      float whistlePowerMax = 0.25f;
      whistleConfidence = (std::min(whistleBandPower, whistlePowerMax) - detectionPower) / (whistlePowerMax - detectionPower);
      PLOT("module:WhistleDetectorRE2017:whistleDetected", 0.9f);
    }
    else
      PLOT("module:WhistleDetectorRE2017:whistleDetected", 0.1f);

    PLOT("module:WhistleDetectorRE2017:whistleConfidence", whistleConfidence);
  }

  PLOT("module:WhistleDetectorRE2017:sufficientSamples", sufficientSamplesForDetection);

  // if no whistle was detected, then we can consider the current window to
  // be background noise and can update the background noise estimate accordingly
  if (!whistleDetected)
  {
    // LOGI("monoNoiseMagnitude\n");
    monoNoiseMagnitude.update(magnitudeSpectrum);

    float sumNoiseMagnitude = 0.f;
    for (int i = 0; i < spectrumSize; i++)
      sumNoiseMagnitude += monoNoiseMagnitude.smoothed[i];
    // avgNoiseMagnitude /= spectrumSize;

    PLOT("module:WhistleDetectorRE2017:sumNoiseMagSmoothed", sumNoiseMagnitude * 1);
  }


  return 0;
}
