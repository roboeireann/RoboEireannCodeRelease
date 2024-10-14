/**
 * @file TFLiteHelperCopy.cpp
 *
 * Help with using TensorFlow-lite C api
 * Based on code kindly provided by Tobias Kalbitz / NaoTeam HTWK
 * Fixed sine ussyes from the original code
 * 
 * @author James Petri
 */

#include "TFLiteHelperCopy.h"

#include "Platform/BHAssert.h"
#include "Tools/TextLogging.h"
#include "Platform/File.h"

#include <cstdio>
#include <tensorflow/lite/c/c_api.h>


DECL_TLOGGER(tlogger, "TFLiteHelperCopy", TextLogging::INFO);

TFLiteHelperCopy::~TFLiteHelperCopy() 
{
  TfLiteInterpreterDelete(interpreter); 
}

void TFLiteHelperCopy::loadModelFromFile(std::string file, std::vector<int> inputDims, int numThreads)
{
  TfLiteModel *model = TfLiteModelCreateFromFile(file.c_str());
  VERIFY(model != nullptr);

  TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
  VERIFY(options != nullptr);
  TfLiteInterpreterOptionsSetNumThreads(options, numThreads);

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
  TLOGV(tlogger, "InputTensor: type {}, numDims {} = ", tensorType, numDims);

  for (int32_t i=0; i < numDims; i++)
    TLOGV(tlogger, "{} ", TfLiteTensorDim(inputTensor, i));
  TLOGV(tlogger, ", byteSize {}\n", TfLiteTensorByteSize(inputTensor));

  const TfLiteTensor *outputTensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
  VERIFY(outputTensor != nullptr);
  tensorType = TfLiteTensorType(outputTensor);
  numDims = TfLiteTensorNumDims(outputTensor);
  TLOGV(tlogger, "InputTensor: type {}, numDims {} = ", tensorType, numDims);
  for (int32_t i=0; i < numDims; i++)
    TLOGV(tlogger, "{} ", TfLiteTensorDim(outputTensor, i));
  TLOGV(tlogger, ", byteSize {}\n", TfLiteTensorByteSize(outputTensor));
}

void TFLiteHelperCopy::loadModelFromArray(const void *modelData, size_t length, std::vector<int> inputDims, int numThreads)
{
  TfLiteModel *model = TfLiteModelCreate(modelData, length);
  VERIFY(model != nullptr);

  TfLiteInterpreterOptions *options = TfLiteInterpreterOptionsCreate();
  VERIFY(options != nullptr);
  TfLiteInterpreterOptionsSetNumThreads(options, numThreads);

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

uint8_t *TFLiteHelperCopy::getUint8InputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteUInt8);

  return static_cast<uint8_t*>(TfLiteTensorData(inputTensor)); 
}

int32_t *TFLiteHelperCopy::getInt32InputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteInt32);

  return static_cast<int32_t*>(TfLiteTensorData(inputTensor)); 
}

float *TFLiteHelperCopy::getFloatInputTensor() 
{ 
  TfLiteTensor *inputTensor = TfLiteInterpreterGetInputTensor(interpreter, 0);
  VERIFY(inputTensor != nullptr);
  VERIFY(TfLiteTensorType(inputTensor) == kTfLiteFloat32);

  return static_cast<float*>(TfLiteTensorData(inputTensor)); 
}

std::vector<float> TFLiteHelperCopy::getFloatOutputTensor() 
{ 
    const TfLiteTensor* outputTensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
    VERIFY(outputTensor != nullptr);

    // Log the actual tensor type
    TfLiteType outputType = TfLiteTensorType(outputTensor);
    // TLOGI(tlogger, "Output Tensor Type: {}", outputType);
    
    // Check the tensor type and handle appropriately
    if (outputType == kTfLiteFloat32) {
        // Directly return the float data
        const float* data = static_cast<const float*>(TfLiteTensorData(outputTensor));
        size_t size = TfLiteTensorByteSize(outputTensor) / sizeof(float);
        return std::vector<float>(data, data + size);
    } else if (outputType == kTfLiteInt32) {
        // Convert int32 data to float
        const int32_t* data = static_cast<const int32_t*>(TfLiteTensorData(outputTensor));
        size_t size = TfLiteTensorByteSize(outputTensor) / sizeof(int32_t);
        std::vector<float> floatData(size);
        for (size_t i = 0; i < size; ++i) {
            floatData[i] = static_cast<float>(data[i]);
        }
        return floatData;
    } else if (outputType == kTfLiteUInt8) {
        // Convert uint8 data to float using a default quantization scheme
        const uint8_t* data = static_cast<const uint8_t*>(TfLiteTensorData(outputTensor));
        size_t size = TfLiteTensorByteSize(outputTensor) / sizeof(uint8_t);

        // Default quantization scheme: assuming [0, 255] maps to [0.0, 1.0]
        std::vector<float> floatData(size);
        for (size_t i = 0; i < size; ++i) {
            floatData[i] = static_cast<float>(data[i]) / 255.0f;
        }
        return floatData;
    }else {
        TLOGE(tlogger, "Unsupported output tensor type: {}", outputType);
        VERIFY(false && "Unsupported output tensor type");
        return std::vector<float>(); // Return an empty vector on unsupported type
    }
  }
// const float *TFLiteHelperCopy::getFloatOutputTensor() 
// { 
//   const TfLiteTensor *outputTensor = TfLiteInterpreterGetOutputTensor(interpreter, 0);
  
//   // Log the actual tensor type
//   TfLiteType outputType = TfLiteTensorType(outputTensor);
//   TLOGI(tlogger, "Output Tensor Type: {}", outputType);

//   VERIFY(outputTensor != nullptr);
//   VERIFY(TfLiteTensorType(outputTensor) == kTfLiteFloat32);

//   return static_cast<const float*>(TfLiteTensorData(outputTensor));
// }

size_t TFLiteHelperCopy::getInputTensorByteSize()
{
  return TfLiteTensorByteSize(TfLiteInterpreterGetInputTensor(interpreter, 0));
}

size_t TFLiteHelperCopy::getOutputTensorByteSize()
{
  return TfLiteTensorByteSize(TfLiteInterpreterGetOutputTensor(interpreter, 0));
}

void TFLiteHelperCopy::execute() 
{
  VERIFY(TfLiteInterpreterInvoke(interpreter) == kTfLiteOk); 
}
