/**
 * @file: WhistleDetectionRE2017.h
 *
 * This a version of the RoboEireann 2017 Whistle Detector adapted to the
 * BH2021 framework
 * 
 * @author: Rudi Villing
 * @author: Shauna Recto
 */

#pragma once


// #include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/GameInfo.h"
// #include "Representations/Communication/RobotInfo.h"
// #include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h" // Used for skipping whistle detection after kick
#include "Tools/Debugging/DebugImages.h" // Used for the Spectrogram
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

#include "Tools/TextLogging.h"

// #include <stdint.h>

#include <fftw3.h>



MODULE(WhistleDetectorRE2017,
{,
  USES(GameInfo),
  REQUIRES(AudioData),
//   REQUIRES(BHumanMessageOutputGenerator),
//   REQUIRES(DamageConfigurationHead),
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
//   REQUIRES(RawGameInfo),
//   REQUIRES(RobotInfo),
  PROVIDES(Whistle),
  LOADS_PARAMETERS(
  {,
    // (int)(128) fftSize, ///< the size of the FFT to use for analysis
    // (int)(128) fftAdvance, ///< the number of samples to step forward for the next FFT (<fftSize means overlap, >fftSize means leave gaps)
    // (unsigned)(1024) bufferSize, ///< The max number of samples (per channel) that we will process in one tick (extra will be discarded).

    // (float)(0.1) detectionPower, ///< average power levels within the "whistle" band above this threshold will be detected as a whistle
    // (float)(3500) minFreqHz, ///< minimum frequency of whistle band
    // (float)(10000) maxFreqHz, ///< max frequency of whistle band
    // (unsigned)(5000) noiseAvgMs, ///< approx ms to average (exp averager time constant) background noise for spectral subtraction
    // (unsigned)(300) audioAvgMs, ///< approx ms to average (exp averager time constant) audio for detection

    (int) fftSize, ///< the size of the FFT to use for analysis
    (int) fftAdvance, ///< the number of samples to step forward for the next FFT (<fftSize means overlap, >fftSize means leave gaps)
    (unsigned) bufferSize, ///< The max number of samples (per channel) that we will process in one tick (extra will be discarded).

    (float) detectionPower, ///< average power levels within the "whistle" band above this threshold will be detected as a whistle
    (float) minFreqHz, ///< minimum frequency of whistle band
    (float) maxFreqHz, ///< max frequency of whistle band
    (unsigned) noiseAvgMs, ///< approx ms to average (exp averager time constant) background noise for spectral subtraction
    (unsigned) audioAvgMs, ///< approx ms to average (exp averager time constant) audio for detection
    (int) minAnnotationIntervalMs, /**< The minimum time between annotations announcing a detected whistle. */
  }),
});


class WhistleDetectorRE2017 : public WhistleDetectorRE2017Base
{
public:
  WhistleDetectorRE2017();
  ~WhistleDetectorRE2017();

private:
  struct Smoothed
  {
    unsigned nAvg = 10;
    unsigned n = 0;
    std::vector<float> smoothed;

    void setNumAverage(unsigned nAvgIn) { nAvg = nAvgIn; }
    void reset() { n = 0; }
    void resize(size_t size, float val=0) { smoothed.resize(size, val); }

    void update(float* input);
  };

  std::vector<RingBuffer<AudioData::Sample>> channelBuffers; ///< Sample buffers for all channels.
  RingBuffer<AudioData::Sample> monoBuffer; ///< Mono mixdown buffer.

  std::vector<float> hannWindow;

  float *samples; ///< the samples that will be used with the FFT 
  fftwf_complex *spectrum; ///< the spectrum of the samples
  fftwf_plan fft; ///< The plan to compute the FFT.

  bool initExtraComplete;

  using FeatureVec = std::vector<float>;

  int spectrumSize;
  Smoothed monoSignalMagnitude; // spectrum of signal
  Smoothed monoSubtractedMagnitude; // subtracted spectrum
  Smoothed monoNoiseMagnitude; // noise spectrum

  Image<PixelTypes::Edge2Pixel> magSpectrogram; ///< Spectrogram of unfiltered spectrum
  Image<PixelTypes::Edge2Pixel> subSpectrogram; ///< Spectrogram of filtered spectrum

  // index of FFT bin corresponding to minFreqHz and maxFreqHz
  int minFFTBin;
  int maxFFTBin;

  unsigned framesSinceOwnSoundPlaying = 0;


  void initExtra(); // additional initialisation during first tick

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theWhistle The representation updated.
   */
  void update(Whistle& theWhistle) override;

  /// helper - process the audio to detect whistles
  void processAudio(Whistle& theWhistle, bool detectWhistles);

  /**
   * Process mono mixdown of all channels to detect the whistle
   * @param start the oldest index (from back of the ringbuffer) of the window to process
   * @param detectWhistles 
   * @returns confidence of detection (0 if not detected)
   */
  float processWindowMonoMix(size_t start);


  inline float clampLegal(float audioSample)
  {
    if (-1.f < audioSample && audioSample < 1.f)
      return audioSample;
    else if (audioSample < -1.f)
      return -1.f;
    else if (audioSample > 1.f)
      return 1.f;
    else // nans, infs, etc
      return 0.f;
  }
};


