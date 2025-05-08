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

#include "tensorrt_net.h"

#define USE_FP16


using namespace nvinfer1;


static const bool isBuildEngine = false;

static TRTLogger tensorRTLogger;


TensorRTNet::TensorRTNet(const std::string &engineFile, const bool &isBuildEngine) {
    std::cout << "generate" << std::endl;
    std::ifstream fs(engineFile);
    if (fs.is_open()) {
        // deserialize engine
        DeserializeTRTEngine(engineFile);
    } else {
        std::cout << "engine was not built" << std::endl;
    }
}

void TensorRTNet::DeserializeTRTEngine(const std::string &engineFile) {
    auto start = std::chrono::system_clock::now();
    std::cout << engineFile << std::endl;

    char *trtModelStream{nullptr};
    size_t size{0};
    std::ifstream file(engineFile, std::ios::binary);
    if (file.good()) {
        file.seekg(0, file.end);
        size = file.tellg();
        file.seekg(0, file.beg);
        trtModelStream = new char[size];
        assert(trtModelStream);
        file.read(trtModelStream, size);
        file.close();
    }

    //static float prob[outputSize];

    //runtime = createInferRuntime(gLogger);
    runtime = createInferRuntime(trtLogger);
    assert(runtime != nullptr);
    engine = runtime->deserializeCudaEngine(trtModelStream, size);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    delete[] trtModelStream;

    auto end = std::chrono::system_clock::now();
    float totaltime = std::chrono::duration<float, std::milli>(end - start).count();
    std::cout << "Engine loading time: " << totaltime << " ms." << std::endl;

}

void TensorRTNet::Build() {

}

void TensorRTNet::Infer(float *input, float *output, int batchSize, char* inputName, char* outputName) {
    const ICudaEngine &engine = context->getEngine();

    // Pointers to input and output device buffers to pass to engine.
    // Engine requires exactly IEngine::getNbBindings() number of buffers.
    assert(engine.getNbBindings() == 2);
    void *buffers[2];
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine.getBindingIndex(inputName);
    const int outputIndex = engine.getBindingIndex(outputName);



   //nvinfer1::Dims3 output_dims = static_cast<nvinfer1::Dims3&&>(engine.getBindingDimensions(outputIndex));

    // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[inputIndex], batchSize * 3 * CameraDetector::INPUT_H * CameraDetector::INPUT_W * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], batchSize * CameraDetector::OUTPUT_SIZE * sizeof(float)));

    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));

    int size = sizeof *input/sizeof(float);



    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CHECK(cudaMemcpyAsync(buffers[inputIndex], input, batchSize * 3 * CameraDetector::INPUT_H * CameraDetector::INPUT_W * sizeof(float),
                          cudaMemcpyHostToDevice, stream));

    context->enqueue(batchSize, buffers, stream, nullptr);

    CHECK(cudaMemcpyAsync(output, buffers[outputIndex], batchSize * CameraDetector::OUTPUT_SIZE * sizeof(float),
                          cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
}

void TensorRTNet::Infer(std::vector<float> &input, std::vector<int> &output, int batchSize, char *inputName, char *outputName)
{

    const ICudaEngine &engine = context->getEngine();

    // Pointers to input and output device buffers to pass to engine.
    // Engine requires exactly IEngine::getNbBindings() number of buffers.
    assert(engine.getNbBindings() == 2);
    std::string in_name = engine.getBindingName(0);
    //void *buffers[2];
    std::vector<void*> buffers(2);
    // In order to bind the buffers, we need to know the names of the input and output tensors.
    // Note that indices are guaranteed to be less than IEngine::getNbBindings()
    const int inputIndex = engine.getBindingIndex(inputName);
    const int outputIndex = engine.getBindingIndex(outputName);


    nvinfer1::Dims3 output_dims =
            static_cast<nvinfer1::Dims3&&>(engine.getBindingDimensions(outputIndex));

    const int in_size{static_cast<int>(input.size())};

    int out_size = batchSize * output_dims.d[1] * output_dims.d[2];
    //std::vector<int> res(out_size);


    // Create GPU buffers on device
    CHECK(cudaMalloc(&buffers[inputIndex], in_size * sizeof(float)));
    CHECK(cudaMalloc(&buffers[outputIndex], out_size * sizeof(int)));

    // Create stream
    cudaStream_t stream;
    CHECK(cudaStreamCreate(&stream));


    // DMA input batch data to device, infer on the batch asynchronously, and DMA output back to host
    CHECK(cudaMemcpyAsync(buffers[inputIndex], &input[inputIndex], in_size * sizeof(float),
                          cudaMemcpyHostToDevice, stream));



    context->enqueueV2(&buffers[inputIndex], stream, nullptr);

    CHECK(cudaMemcpyAsync(&output[inputIndex], buffers[outputIndex], out_size * sizeof(int),
                          cudaMemcpyDeviceToHost, stream));
    cudaStreamSynchronize(stream);

    // Release stream and buffers
    cudaStreamDestroy(stream);
    CHECK(cudaFree(buffers[inputIndex]));
    CHECK(cudaFree(buffers[outputIndex]));
    //output = res;

}


bool TensorRTNet::Init(const std::map<std::string, std::vector<int> > &shapes) {

    return true;
}


std::map<std::string, Weights> TensorRTNet::LoadWeights(const std::string file) {
    std::map<std::string, Weights> weightMap;

    std::ifstream input(file);
    assert(input.is_open() && "Unable to load weight file.");

    // Read number of weight blobs
    int32_t count;
    input >> count;
    assert(count > 0 && "Invalid weight map file.");

    while (count--)
    {
        Weights wt{DataType::kFLOAT, nullptr, 0};
        uint8_t size;

        // Read name and type of blob
        std::string name;
        input >> name >> std::dec >> size;
        wt.type = DataType::kFLOAT;

        // Load blob
        uint32_t* val = reinterpret_cast<uint32_t*>(malloc(sizeof(val) * size));

        for(uint32_t x = 0, y = size; x < y; ++x){
            input >> std::hex >> val[x];
        }
        wt.values = val;

        wt.count = size;
        weightMap[name] = wt;

    }
    return weightMap;
}

void TensorRTNet::APIToTRTEngine(unsigned int maxBatchSize, IHostMemory **modelStream,
                              ICudaEngine* (*CreateNetwork)(unsigned int, IBuilder *, IBuilderConfig *, DataType)) {
    IBuilder* builder = createInferBuilder(tensorRTLogger);
    IBuilderConfig* config = builder->createBuilderConfig();

    // Create model to populate the network, then set the outputs and create an engine
    ICudaEngine* engine = CreateNetwork(maxBatchSize, builder, config, DataType::kFLOAT);
    assert(engine != nullptr);

    // Serialize the engine
    (*modelStream) = engine->serialize();

    // Close everything down
    engine->destroy();
    builder->destroy();

}


void TensorRTNet::ONNXToTRTEngine(const int batchSize, const std::string &onnxFile,
                                  nvinfer1::IHostMemory *&trtModelStream)  {

    int verbosity = (int)nvinfer1::ILogger::Severity::kWARNING;

    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(tensorRTLogger);
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();

    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicitBatch);

    auto parser = nvonnxparser::createParser(*network, tensorRTLogger);

    if (!parser->parseFromFile(onnxFile.c_str(), verbosity))
    {
        std::string msg("failed to parse onnx file");
        tensorRTLogger.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
        exit(EXIT_FAILURE);
    }

    // Build the engine
    builder->setMaxBatchSize(batchSize);
    //builder->setMaxWorkspaceSize(1 << 20);
    config->setMaxWorkspaceSize(1 << 20);


    //nvinfer1::ICudaEngine* engine = builder->buildCudaEngine(*network);

    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network,*config);


    parser->destroy();

    // serialize the engine, then close everything down
    trtModelStream = engine->serialize();
    engine->destroy();
    network->destroy();
    builder->destroy();
}

void TensorRTNet::DeserializeONNXEngine(const std::string &onnxFile) {

    nvinfer1::IHostMemory* stream{ nullptr };
    ONNXToTRTEngine(1, onnxFile, stream);

    if (stream == nullptr)
    {
       gLogFatal<< "Failed to load ONNX file " << std::endl;
    }

    // deserialize the engine
    runtime = nvinfer1::createInferRuntime(tensorRTLogger);

    if (runtime == nullptr)
    {
        gLogFatal<<"Failed to create TensorRT Runtime object."<<std::endl;
    }
    engine =
            runtime->deserializeCudaEngine(stream->data(), stream->size(), nullptr);

    if (engine == nullptr)
    {
        gLogFatal << "Failed to create TensorRT Engine." << std::endl;
    }
    stream->destroy();

    context = engine->createExecutionContext();

    if (context == nullptr)
    {
        gLogFatal << "Failed to create TensorRT Execution Context." << std::endl;;
    }
}



