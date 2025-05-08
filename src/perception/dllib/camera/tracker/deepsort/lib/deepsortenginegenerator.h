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
#include <NvInfer.h>
#include <NvOnnxParser.h>

using namespace nvinfer1;

const int IMG_HEIGHT = 128;
const int IMG_WIDTH = 64;
const int MAX_BATCH_SIZE = 128;
const std::string INPUT_NAME("input");

class DeepSortEngineGenerator {
public:
    DeepSortEngineGenerator(ILogger* gLogger);
    ~DeepSortEngineGenerator();

public:
    void setFP16(bool state);
    void createEngine(std::string onnxPath, std::string enginePath);

private: 
    std::string modelPath, engingPath;
    ILogger* gLogger;  
    bool useFP16; 
};

