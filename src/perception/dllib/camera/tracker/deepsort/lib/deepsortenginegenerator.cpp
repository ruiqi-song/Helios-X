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

#include "deepsortenginegenerator.h"
#include "assert.h"
#include <memory.h> 
#include <fstream>

DeepSortEngineGenerator::DeepSortEngineGenerator(ILogger* gLogger) {
    this->gLogger = gLogger;
}

DeepSortEngineGenerator::~DeepSortEngineGenerator() {

}

void DeepSortEngineGenerator::setFP16(bool state) {
    this->useFP16 = state;
}

void DeepSortEngineGenerator::createEngine(std::string onnxPath, std::string enginePath) {
    // Load onnx model
    auto builder = createInferBuilder(*gLogger);
    assert(builder != nullptr);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    assert(network != nullptr);
    auto config = builder->createBuilderConfig();
    assert(config != nullptr);

    auto profile = builder->createOptimizationProfile();
    Dims dims = Dims4{1, 3, IMG_HEIGHT, IMG_WIDTH};
    profile->setDimensions(INPUT_NAME.c_str(),
                OptProfileSelector::kMIN, Dims4{1, dims.d[1], dims.d[2], dims.d[3]});
    profile->setDimensions(INPUT_NAME.c_str(),
                OptProfileSelector::kOPT, Dims4{MAX_BATCH_SIZE, dims.d[1], dims.d[2], dims.d[3]});
    profile->setDimensions(INPUT_NAME.c_str(),
                OptProfileSelector::kMAX, Dims4{MAX_BATCH_SIZE, dims.d[1], dims.d[2], dims.d[3]});
    config->addOptimizationProfile(profile);

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, *gLogger);
    assert(parser != nullptr);
    auto parsed = parser->parseFromFile(onnxPath.c_str(), static_cast<int>(ILogger::Severity::kWARNING));
    assert(parsed);
    if (useFP16) config->setFlag(BuilderFlag::kFP16);
    config->setMaxWorkspaceSize(1 << 20);
    ICudaEngine* engine = builder->buildEngineWithConfig(*network, *config);

    // Serialize model and save engine
    IHostMemory* modelStream = engine->serialize();
    std::string serializeStr;
    std::ofstream serializeOutputStream;
    serializeStr.resize(modelStream->size());
    memcpy((void*)serializeStr.data(), modelStream->data(), modelStream->size());
    serializeOutputStream.open(enginePath);
    serializeOutputStream << serializeStr;
    serializeOutputStream.close();
}