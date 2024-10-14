/**
 * @file TFLiteHelperCopy.h
 *
 * Help with using TensorFlow-lite C api
 * Based on code kindly provided by Tobias Kalbitz / NaoTeam HTWK
 * Fixed sine ussyes from the original code
 * 
 * @author James Petri
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct TfLiteInterpreter;

class TFLiteHelperCopy
{
public:
  TFLiteHelperCopy() = default;
  ~TFLiteHelperCopy();

  TFLiteHelperCopy(TFLiteHelperCopy &h) = delete;
  TFLiteHelperCopy(TFLiteHelperCopy &&h) = delete;
  TFLiteHelperCopy &operator=(const TFLiteHelperCopy &) = delete;
  TFLiteHelperCopy &operator=(TFLiteHelperCopy &&) = delete;

  void loadModelFromFile(std::string file, std::vector<int> inputDims, int numThreads = 1);
  void loadModelFromArray(const void *modelData, size_t length, std::vector<int> inputDims, int numThreads = 1);

  uint8_t *getUint8InputTensor();
  int32_t *getInt32InputTensor();
  float *getFloatInputTensor();
  std::vector<float> getFloatOutputTensor();
  //const float *getFloatOutputTensor();

  size_t getInputTensorByteSize();
  size_t getOutputTensorByteSize();

  void execute();
  float *getResult();

private:
  TfLiteInterpreter *interpreter = nullptr;
};
