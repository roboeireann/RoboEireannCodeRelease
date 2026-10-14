/**
 * @file: REWhistleDetector.h
 *
 * This is the RoboEireann whistle detector source ported the
 * BH2025 framework.
 *
 * @author: Shauna Recto
 */

#pragma once

#include "Framework/Module.h"
#include "Math/RingBuffer.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"

#include <CompiledNN2ONNX/CompiledNN.h>

#include <fftw3.h> // Library used for obtaining FFTs
#include <string>

/* Module Declaration */
MODULE(REWhistleDetector,
{,
  REQUIRES(AudioData),
  REQUIRES(FrameInfo),
  PROVIDES(Whistle),

  LOADS_PARAMETERS(
  {,
    (std::string) whistleDetName,
    (int) detectionDelayMs,
    (int) fftSize,
    (int) fftStep,
    (unsigned) bufferSize,
    (int) numChannels,
    (float) whistleConfidenceThresh,
    (float) sampleGain,
    (unsigned) noiseAvgLenMs,
  }),
});

class REWhistleDetector : public REWhistleDetectorBase {
public:
  REWhistleDetector();
  ~REWhistleDetector();

private:

  bool extraInit = false;

  class NoiseFilter {
  private:
    unsigned int numSample = 0;
    unsigned int maxNumSample;

  public:
    std::vector<float> spectProfile;

    NoiseFilter() = default;
    ~NoiseFilter() = default;
    void setParams(unsigned, unsigned);
    void reset() {numSample = 0;}
    void update(const std::vector<float>&);
  };

  using SampleBuffer = RingBuffer<AudioData::Sample>;
  std::vector<SampleBuffer> channelBuffers;  // Sample buffers for all channels.
  SampleBuffer monoBuffer;         // Mono mixdown buffer.

  /* FFTW related members */
  float* samples = nullptr;             // The samples that will be used with the FFT
  fftwf_complex* spectrum = nullptr;     // the spectrum of the samples
  fftwf_plan fft = nullptr;              // The plan to compute the FFT.

  /* magSpectrums Contains numFFTs FFTs of size spectrumSize */
  RingBuffer<std::vector<float>> magSpectrums;
  int spectrumSize;
  int maxFFTs;

  NoiseFilter noisefilter;

  /* Other FFT Related members */
  std::vector<float> hannWindow;

  /* CompiledNN members */
  NeuralNetworkONNX::CompiledNN whistleDetector;
  std::unique_ptr<NeuralNetworkONNX::Model> model = nullptr;

  void update(Whistle&);              /* Runs update cycle for WD */
  void processAudio(Whistle&, bool);  /* Processes Audio for inferrence */
  float inferenceCheck();             /* Checks audio contents against NN for whistles */
};
