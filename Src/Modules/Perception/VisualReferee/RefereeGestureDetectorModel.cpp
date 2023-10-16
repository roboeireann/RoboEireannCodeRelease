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
#include "Tools/ImageProcessing/ColorModelConversions.h"

#include "Tools/TextLogging.h"

#include <tensorflow/lite/c/c_api.h>
#include <fmt/core.h>
// #include "tensorflow/lite/kernels/register.h"
// #include "tensorflow/lite/model.h"
// #include "tensorflow/lite/tools/gen_op_registration.h"


DECL_TLOGGER(tlogger, "RefereeGestureDetectorModel", TextLogging::INFO);

RefereeGestureDetectorModel::RefereeGestureDetectorModel()
{
  inferenceThread.setPriority(0);
  inferenceThread.start(this, &RefereeGestureDetectorModel::inferenceThreadMain);
}


RefereeGestureDetectorModel::~RefereeGestureDetectorModel()
{
  inferenceThread.stop();
}


bool RefereeGestureDetectorModel::tryStartInference(const CameraImage& cameraImage)
{
  SYNC;
  if (state != IDLE)
    return false;

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

      TLOGD(tlogger, "{},{}, i={}\n", x,y,i);

      i += 3;
    }
  } 

  state = PROCESSING;

  return true; // image submitted for processing
}

bool RefereeGestureDetectorModel::tryGetInferenceOutput(DetectedGesture::GestureOneHot &detectedGesture)
{
  // static int everyN = 1;
  static int count = 0;

  SYNC;
  if (state == RESULT)
  {
    // if (--everyN == 0)
    // {
    //   everyN = 10;
      TLOGI(tlogger, "{} -- {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}\n",
                 count++,
                 gestureOutput[0], gestureOutput[1], gestureOutput[2], gestureOutput[3], gestureOutput[4], gestureOutput[5],
                 gestureOutput[6], gestureOutput[7], gestureOutput[8], gestureOutput[9], gestureOutput[10], gestureOutput[11],
                 gestureOutput[12]);
      TLOGI(tlogger, "         {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}\n",
                 gestureInput[0], gestureInput[1], gestureInput[2], gestureInput[3], gestureInput[4], gestureInput[5],
                 gestureInput[6], gestureInput[7]);
    // }

    int best = 0;
    for (int i=0; i<12; i++)
      if (gestureOutput[i] > gestureOutput[best])
        best = i;

    detectedGesture[best] = 1;

    state = IDLE;
    return true;
  }

  return false;
}


void RefereeGestureDetectorModel::inferenceThreadMain()
{
  TLOGI(tlogger, "Hello from TensorFlow C library version {}\n", TfLiteVersion());

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

  {
    SYNC;
    state = IDLE;
  }

  // thread loop
  while (inferenceThread.isRunning())
  {
    bool process = false;
    {
      SYNC;
      if (state == PROCESSING)
        process = true;
    }

    if (process)
    {
      moveNetHelper.execute();

      memcpy(gestureInput, moveNetOutput, 17*3*sizeof(float));

      gestureInput = gestureHelper.getFloatInputTensor();
      
      gestureHelper.execute();

      TLOGI(tlogger, "new gestureOutput\n");

      gestureOutput = gestureHelper.getFloatOutputTensor();
      VERIFY(gestureOutput != nullptr);

      SYNC;
      state = RESULT;
    }
    else
      inferenceThread.sleep(100);
  }
}