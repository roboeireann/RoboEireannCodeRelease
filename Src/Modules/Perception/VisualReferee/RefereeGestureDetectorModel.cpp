/**
 * @file RefereeGestureDetectorModel.cpp
 *
 * Executes the inference model in another thread to limit the impact
 * on the real-time deadlines of the perception thread
 *
 * @author Rudi Villing
 */

#include "RefereeGestureDetectorModel.h"
#include "TFLiteHelper.h"

#include "Platform/File.h"
#include "Platform/Thread.h"

#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/TextLogging.h"

#include <tflite_c_api.h>
#include <tflite_c_api_xnnpack_delegate.h>
#include <fmt/core.h>

#include <mutex>
#include <condition_variable>



DECL_TLOGGER(tlogger, "RefereeGestureDetectorModel", TextLogging::INFO);


// internal thread related implementation details are kept out of the header file

static Thread inferenceThread; ///< thread that will do the asynchronous inference
static std::mutex mtx;
static std::condition_variable cv;

#define SYNCHRONIZED(_m) std::lock_guard<std::mutex> lockg(_m)
#define SYNC_COND(_m, _lck) std::unique_lock<std::mutex> _lck(_m)


namespace // make the State type "private" to this file
{
  enum State { INITIALISING, IDLE, PREPARING_INPUT, DO_INFERENCE, RESULT_AVAILABLE, RETRIEVE_RESULT };
}
static State state = INITIALISING;


RefereeGestureDetectorModel::RefereeGestureDetectorModel()
{
  inferenceThread.setPriority(0);
  inferenceThread.start(this, &RefereeGestureDetectorModel::inferenceThreadMain);
}


RefereeGestureDetectorModel::~RefereeGestureDetectorModel()
{
  inferenceThread.announceStop();
  cv.notify_one();
  inferenceThread.stop();
  TLOGI(tlogger, "Inference thread exited and joined");
}


bool RefereeGestureDetectorModel::tryStartInference(const CameraImage& cameraImage)
{
  {
    SYNCHRONIZED(mtx);
    if (state != IDLE)
      return false;
    else
      state = PREPARING_INPUT;
  }

  // FIXME: process image - this is a really slow and naive copy/conversion
  // of the image

  ASSERT(cameraImage.height == 480);

  int i = 0;
  for (int y = 48; y < 48 + 384; y += 2)
  {
    for (int x = 128; x < 128 + 384; x += 2)
    {
      const PixelTypes::YUYVPixel& yuyv = cameraImage(x / 2, y);

      ColorModelConversions::fromYUVToRGB(yuyv.y(x), yuyv.u, yuyv.v, moveNetInput[i], moveNetInput[i+1], moveNetInput[i+2]);

      // TLOGV(tlogger, "{},{}, i={}", x,y,i);

      i += 3;
    }
  } 

  // input prepared, so process it

  {
    SYNCHRONIZED(mtx);
    state = DO_INFERENCE;
  }
  cv.notify_one(); // wake up the inference thread

  return true; // image submitted for processing
}


bool RefereeGestureDetectorModel::tryGetInferenceOutput(DetectedGesture::GestureOneHot &detectedGesture)
{
  // static int everyN = 1;
  static int count = 0;

  {
    SYNCHRONIZED(mtx);
    if (state == RESULT_AVAILABLE)
      state = RETRIEVE_RESULT;
    else
      return false; // no result available
  }

  // if we get this far, there was a result available

  TLOGD(tlogger, "{} -- {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}",
              count++,
              gestureOutput[0], gestureOutput[1], gestureOutput[2], gestureOutput[3], gestureOutput[4], gestureOutput[5],
              gestureOutput[6], gestureOutput[7], gestureOutput[8], gestureOutput[9], gestureOutput[10], gestureOutput[11],
              gestureOutput[12]);
  TLOGD(tlogger, "         {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n",
              gestureInput[0], gestureInput[1], gestureInput[2], gestureInput[3], gestureInput[4], gestureInput[5],
              gestureInput[6], gestureInput[7]);

  int best = 0;
  for (int i=0; i<12; i++)
    if (gestureOutput[i] > gestureOutput[best])
      best = i;

  detectedGesture[best] = 1;

  {
    SYNCHRONIZED(mtx);
    state = IDLE;
  }
  return true;
}


void RefereeGestureDetectorModel::inferenceThreadMain()
{
  // NOTE: the state is INITIALISING until the inference thread changes it

  Thread::nameCurrentThread("RefGestureInference");

  TLOGI(tlogger, "***** Inference thread initialising, TensorFlow C library version {}", TfLiteVersion());

  TFLiteHelper moveNetHelper;

  const int batchSize = 1;
  const int height = 192;
  const int width = 192;
  const int channels = 3;
  std::string filepath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/movenet_lightning.tflite";
  moveNetHelper.loadModelFromFile(filepath, {batchSize,height,width,channels});

  moveNetInput = moveNetHelper.getUint8InputTensor();
  moveNetOutput = moveNetHelper.getFloatOutputTensor();

  TFLiteHelper gestureHelper;

  // const int bs2 = 1;
  // const int height = 192;
  // const int width = 192;
  // const int channels = 3;
  filepath = std::string(File::getBHDir()) + "/Config/NeuralNets/VisualReferee/gesture_classifier.tflite";
  gestureHelper.loadModelFromFile(filepath, {1,1,17,3});

  gestureInput = gestureHelper.getFloatInputTensor();

  gestureHelper.execute();

  gestureOutput = gestureHelper.getFloatOutputTensor();
  VERIFY(gestureOutput != nullptr);

  // change state from INITIALISING to IDLE to indicate this thread is ready to provide service
  {
    SYNCHRONIZED(mtx);
    state = IDLE;
  }

  // thread loop
  while (inferenceThread.isRunning())
  {
    {
      SYNC_COND(mtx, lock);
      if (state != DO_INFERENCE)
      {
        // wait for a signal that DO_INFERENCE has been requested, or the inferenceThread should stop, or we've timed out
        cv.wait_for(lock, std::chrono::seconds(1), []{ return (state == DO_INFERENCE) || !inferenceThread.isRunning(); });
        // should the thread stop running?
        if (!inferenceThread.isRunning())
          break;
        else if (state != DO_INFERENCE) // if we timed out we should just repeat the outer loop
          continue;
      }
    }

    // if we get this far, state == DO_INFERENCE, so do it
    moveNetHelper.execute();

    memcpy(gestureInput, moveNetOutput, 17*3*sizeof(float));

    gestureInput = gestureHelper.getFloatInputTensor();
    
    gestureHelper.execute();

    gestureOutput = gestureHelper.getFloatOutputTensor();
    VERIFY(gestureOutput != nullptr);

    TLOGD(tlogger, "new gestureOutput result available");

    {
      SYNCHRONIZED(mtx);
      state = RESULT_AVAILABLE;
    }
  }

  TLOGI(tlogger, "***** Inference thread exiting");
}