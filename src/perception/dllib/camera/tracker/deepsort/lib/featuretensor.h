/******************************************************************************
 * Copyright 2022 The Helios-X Authors. All Rights Reserved.
 * Author: Ricky Song
 * Time: 2022-1-24
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include "model.hpp"
#include "datatype.h"
#include "cuda_runtime_api.h"

using std::vector;
using nvinfer1::ILogger;

class FeatureTensor {
public:
    FeatureTensor(const int maxBatchSize, const cv::Size imgShape, const int featureDim, int gpuID, ILogger* gLogger);
    ~FeatureTensor();

public:
    bool getRectsFeature(const cv::Mat& img, DETECTIONS& det);
    bool getRectsFeature(DETECTIONS& det);
    void loadEngine(std::string enginePath);
    void loadOnnx(std::string onnxPath);
    int getResult(float*& buffer);
    void doInference(vector<cv::Mat>& imgMats);

private:
    void initResource();
    void doInference(float* inputBuffer, float* outputBuffer);
    void mat2stream(vector<cv::Mat>& imgMats, float* stream);
    void stream2det(float* stream, DETECTIONS& det);

private:
    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    const int maxBatchSize;
    const cv::Size imgShape;
    const int featureDim;

private:
    int curBatchSize;
    const int inputStreamSize, outputStreamSize;
    bool initFlag;
    float* const inputBuffer;
    float* const outputBuffer;
    int inputIndex, outputIndex;
    void* buffers[2];
    cudaStream_t cudaStream;
    // BGR format
    float means[3], std[3];
    const std::string inputName, outputName;
    ILogger* gLogger;
};

