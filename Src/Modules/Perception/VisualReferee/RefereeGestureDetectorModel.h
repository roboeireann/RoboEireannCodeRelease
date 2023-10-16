/**
 * @file RefereeGestureDetectorModel.h
 *
 * Executes the inference model in another thread to limit the impact
 * on the real-time deadlines of the perception thread
 *
 * @author Rudi Villing
 */

#pragma once

#include "DetectedGesture.h"

// #include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Representations/Infrastructure/CameraImage.h"

class RefereeGestureDetectorModel
{
public:
  using Gesture = DetectedGesture::Gesture;

  RefereeGestureDetectorModel();
  ~RefereeGestureDetectorModel();

  bool tryStartInference(const CameraImage& cameraImage);
  bool tryGetInferenceOutput(DetectedGesture::GestureOneHot &detectedGesture); // check this before tryProcessImage for max concurrency

private:
  DECLARE_SYNC;

  Thread inferenceThread; ///< thread that will do the asynchronous inference
  
  enum { INITIALISING, IDLE, PROCESSING, RESULT } state = INITIALISING;
  uint8_t *moveNetInput = nullptr;
  const float *moveNetOutput = nullptr;
  float *gestureInput = nullptr;
  const float *gestureOutput = nullptr;

  void inferenceThreadMain(); ///< the entry point of the inference thread
};