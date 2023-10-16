/**
 * @file TFLiteHelper.h
 *
 * Help with using TensorFlow-lite C api
 * Based on code kindly provided by Tobias Kalbitz / NaoTeam HTWK
 *
 * @author Rudi Villing
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

struct TfLiteInterpreter;

class TFLiteHelper
{
public:
  TFLiteHelper() = default;
  ~TFLiteHelper();

  TFLiteHelper(TFLiteHelper &h) = delete;
  TFLiteHelper(TFLiteHelper &&h) = delete;
  TFLiteHelper &operator=(const TFLiteHelper &) = delete;
  TFLiteHelper &operator=(TFLiteHelper &&) = delete;

  void loadModelFromFile(std::string file, std::vector<int> inputDims, int numThreads = 1);
  void loadModelFromArray(const void *modelData, size_t length, std::vector<int> inputDims, int numThreads = 1);

  uint8_t *getUint8InputTensor();
  int32_t *getInt32InputTensor();
  float *getFloatInputTensor();
  const float *getFloatOutputTensor();

  size_t getInputTensorByteSize();
  size_t getOutputTensorByteSize();

  void execute();
  float *getResult();

private:
  TfLiteInterpreter *interpreter = nullptr;
};
