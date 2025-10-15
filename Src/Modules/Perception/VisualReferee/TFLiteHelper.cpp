/**
 * @file TFLiteHelper.cpp
 *
 * Help with using TensorFlow-lite C api
 * Based on code kindly provided by Tobias Kalbitz / NaoTeam HTWK
 *
 * @author Rudi Villing
 */

#include "TFLiteHelper.h"

#include "Platform/BHAssert.h"
#include "Tools/TextLogging.h"

#include <cstdio>
#include <tflite_c_api.h>
#include <tflite_c_api_xnnpack_delegate.h>

DECL_TLOGGER(tlogger, "TFLiteHelper", TextLogging::INFO);

TFLiteHelper::~TFLiteHelper() 
{
  TfLiteInterpreterDelete(interpreter); 
  if (delegate) {
    TfLiteXNNPackDelegateDelete(delegate);
  }
}

void TFLiteHelper::loadModelFromFile(std::string file, std::vector<int> inputDims, int numThreads)
{
  TfLiteModel *model = TfLiteModelCreateFromFile(file.c_str());
  VERIFY(model != nullptr);

  // Create XNNPACK delegate
  TfLiteXNNPackDelegateOptions xnnPackDelegateOption = TfLiteXNNPackDelegateOptionsDefault();
  xnnPackDelegateOption.num_threads = numThreads;
  delegate = TfLiteXNNPackDelegateCreate(&xnnPackDelegateOption);
  VERIFY(delegate != nullptr);

  TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
  VERIFY(options != nullptr);
  TfLiteInterpreterOptionsSetNumThreads(options, numThreads);
  TfLiteInterpreterOptionsAddDelegate(options, delegate);

  interpreter = TfLiteInterpreterCreate(model, options);
  VERIFY(interpreter != nullptr);

  TfLiteInterpreterOptionsDelete(options);
  TfLiteModelDelete(model);

  VERIFY(TfLiteInterpreterAllocateTensors(interpreter) == kTfLiteOk);
  VERIFY(TfLiteInterpreterGetInputTensorCount(interpreter) == 1);

  // INPUT TENSOR DEBUG CODE - DO NOT DELETE!!!!
  // if (TfLiteInterpreterGetInputTensorCount(interpreter) > 0) {
  //   TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  //   int32_t numDims = TfLiteTensorNumDims(inputTensor);
  //   TLOGI(tlogger, "Input Tensor numDims: {}", numDims);
  //   for (int32_t i = 0; i < numDims; ++i) {
  //     TLOGI(tlogger, "  Dim {}: {}", i, TfLiteTensorDim(inputTensor, i));
  //   }

  //   // Resize the input tensor
  //   TLOGI(tlogger, "Resizing input tensor to dimensions: {}", inputDims.size());
  //   for (size_t i = 0; i < inputDims.size(); ++i) {
  //     TLOGI(tlogger, "  Input Dim {}: {}", i, inputDims[i]);
  //   }
  //   VERIFY(TfLiteInterpreterResizeInputTensor(interpreter, 0, inputDims.data(), static_cast<int32_t>(inputDims.size())) == kTfLiteOk);

  //   // Check dimensions again after resizing
  //   TLOGI(tlogger, "Input tensor dimensions after resizing:");
  //   numDims = TfLiteTensorNumDims(inputTensor);
  //   for (int32_t i = 0; i < numDims; ++i) {
  //     TLOGI(tlogger, "  Dim {}: {}", i, TfLiteTensorDim(inputTensor, i));
  //   }
  // } else {
  //   TLOGE(tlogger, "No input tensors found!");
  // }

  VERIFY(TfLiteInterpreterGetOutputTensorCount(interpreter) == 1);

  VERIFY(TfLiteInterpreterResizeInputTensor(interpreter, 0, inputDims.data(), static_cast<int32_t>(inputDims.size())) == kTfLiteOk);
  VERIFY(TfLiteInterpreterAllocateTensors(interpreter) == kTfLiteOk);

  // debug: check dimensions
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  TfLiteType tensorType = TfLiteTensorType(inputTensor);
  int32_t numDims = TfLiteTensorNumDims(inputTensor);
  TLOGI(tlogger, "InputTensor: type {}, numDims {} = ", tensorType, numDims);

  for (int32_t i=0; i < numDims; i++)
    TLOGI(tlogger, "{} ", TfLiteTensorDim(inputTensor, i));
  TLOGI(tlogger, ", byteSize {}\n", TfLiteTensorByteSize(inputTensor));

  const TfLiteTensor *outputTensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
  VERIFY(outputTensor != nullptr);
  tensorType = TfLiteTensorType(outputTensor);
  numDims = TfLiteTensorNumDims(outputTensor);
  TLOGI(tlogger, "OutputTensor: type {}, numDims {} = ", tensorType, numDims);
  for (int32_t i=0; i < numDims; i++)
    TLOGI(tlogger, "{} ", TfLiteTensorDim(outputTensor, i));
  TLOGI(tlogger, ", byteSize {}\n", TfLiteTensorByteSize(outputTensor));
}

void TFLiteHelper::loadModelFromArray(const void *modelData, size_t length, std::vector<int> inputDims, int numThreads)
{
  TfLiteModel *model = TfLiteModelCreate(modelData, length);
  VERIFY(model != nullptr);

  TfLiteXNNPackDelegateOptions xnnPackDelegateOption = TfLiteXNNPackDelegateOptionsDefault();
  xnnPackDelegateOption.num_threads = numThreads;
  delegate = TfLiteXNNPackDelegateCreate(&xnnPackDelegateOption);
  VERIFY(delegate != nullptr);

  TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
  VERIFY(options != nullptr);
  TfLiteInterpreterOptionsSetNumThreads(options, numThreads);
  TfLiteInterpreterOptionsAddDelegate(options, delegate);

  interpreter = TfLiteInterpreterCreate(model, options);
  VERIFY(interpreter != nullptr);

  TfLiteInterpreterOptionsDelete(options);
  TfLiteModelDelete(model);

  VERIFY(TfLiteInterpreterAllocateTensors(interpreter) == kTfLiteOk);
  VERIFY(TfLiteInterpreterGetInputTensorCount(interpreter) == 1);
  VERIFY(TfLiteInterpreterGetOutputTensorCount(interpreter) == 1);

  VERIFY(TfLiteInterpreterResizeInputTensor(interpreter, 0, inputDims.data(), static_cast<int32_t>(inputDims.size())) == kTfLiteOk);
  VERIFY(TfLiteInterpreterAllocateTensors(interpreter) == kTfLiteOk);
}

uint8_t *TFLiteHelper::getUint8InputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteUInt8);

  return static_cast<uint8_t*>(TfLiteTensorData(inputTensor)); 
}

int32_t *TFLiteHelper::getInt32InputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteInt32);

  return static_cast<int32_t*>(TfLiteTensorData(inputTensor)); 
}

float *TFLiteHelper::getFloatInputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteFloat32);

  return static_cast<float*>(TfLiteTensorData(inputTensor)); 
}

const float *TFLiteHelper::getFloatOutputTensor() 
{ 
  const TfLiteTensor *outputTensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
  VERIFY(outputTensor != nullptr);
  VERIFY(TfLiteTensorType(outputTensor) == kTfLiteFloat32);

  return static_cast<const float*>(TfLiteTensorData(outputTensor));
}

size_t TFLiteHelper::getInputTensorByteSize()
{
  return TfLiteTensorByteSize(TfLiteInterpreterGetInputTensor(interpreter, 0));
}

size_t TFLiteHelper::getOutputTensorByteSize()
{
  return TfLiteTensorByteSize(TfLiteInterpreterGetOutputTensor(interpreter, 0));
}

void TFLiteHelper::execute() 
{
  VERIFY(TfLiteInterpreterInvoke(interpreter) == kTfLiteOk); 
}
