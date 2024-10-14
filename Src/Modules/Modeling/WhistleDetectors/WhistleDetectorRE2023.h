/**
 * @file: WhistleDetectionRE2023.h
 *
 * This is the new RoboEireann whistle detector designed for the
 * BH2021 framework.
 * 
 * @author: Shauna Recto
 */

#pragma once

/* Internal Headers */
#include "Representations/Communication/GameInfo.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Whistle.h"           // Whistle
#include "Representations/MotionControl/MotionInfo.h"   // Used for skipping whistle detection after kick
#include "Tools/Module/Module.h"                        // For Module declaration and definition
#include "Tools/RingBuffer.h"                           // Ring buffer types

#include <CompiledNN/Model.h>                           // Used to make NN model
#include <CompiledNN/CompiledNN.h>

#include <cmath>
#include <string>
#include <fftw3.h>                                      // Library used for obtaining FFTs

/* Module Declaration */
MODULE(WhistleDetectorRE2023,
{,
  USES(GameInfo),
  REQUIRES(AudioData),
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),

  PROVIDES(Whistle),

  LOADS_PARAMETERS(
  {,
    (std::string) whistleDetName,
    (int) fftSize,
    (int) fftStep,
    (unsigned) bufferSize,
    (float) whistleConfidenceThresh,
    (float) whistleBias,
    (float) sampleGain,
    (int) averagingLength,
  }),
});

class WhistleDetectorRE2023 : public WhistleDetectorRE2023Base
{
public:
  WhistleDetectorRE2023();
  ~WhistleDetectorRE2023();

private:
  std::vector<RingBuffer<AudioData::Sample>> channelBuffers; // Sample buffers for all channels.
  RingBuffer<AudioData::Sample> monoBuffer;                  // Mono mixdown buffer.
  RingBuffer<AudioData::Sample> leftOverBuffer;              // Unused samples for next cycle.

  /* FFTW related members */
  float *samples;          // The samples that will be used with the FFT
  fftwf_complex *spectrum; // the spectrum of the samples
  fftwf_plan fft;          // The plan to compute the FFT.
  /* magSpectrums Contains numFFTs FFTs of size spectrumSize */
  RingBuffer<std::vector<float>> magSpectrums;
  int spectrumSize;

  /* Other FFT Related members */
  std::vector<float> hannWindow;

  /* CompiledNN members */
  NeuralNetwork::CompiledNN whistleDetCompNN;
  std::unique_ptr<NeuralNetwork::Model> whistleDetModel;

  /* Confidence members */
  std::vector<float> whistleConfidences;

  void update(Whistle &) override;    // Updates on WD
  void processAudio(Whistle &, bool); // Manipulates Audio for WD
  float compareAudio();               // Checks audio contents against NN for whistles

  /* Supporting Member functions */
  float clampSample(float, float); // Checks and forces sample to be within range.
};