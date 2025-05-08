//
// Created by ricky on 2022/3/24.
//

#include "centerpoint.h"


CenterPoint::CenterPoint() {

}

CenterPoint::~CenterPoint() {
    GPU_CHECK(cudaFree(deviceIndices));
    //GPU_CHECK(cudaFree(dev_points));
    GPU_CHECK(cudaFree(devicePillars));


    GPU_CHECK(cudaFree( _PBEVIdxs));
    GPU_CHECK(cudaFree( _PPointNumAssigned));
    GPU_CHECK(cudaFree( _PMask));
    GPU_CHECK(cudaFree( _BEVVoxelIdx)); // H * W
    GPU_CHECK(cudaFree( _VPointSum));
    GPU_CHECK(cudaFree( _VRange));
    GPU_CHECK(cudaFree( _VPointNum));

    GPU_CHECK(cudaFree(pfe_buffers_[0]));
    GPU_CHECK(cudaFree(pfe_buffers_[1]));

    GPU_CHECK(cudaFree(dev_scattered_feature_));

    GPU_CHECK(cudaFree(rpn_buffers_[0]));
    GPU_CHECK(cudaFree(rpn_buffers_[1]));
    GPU_CHECK(cudaFree(rpn_buffers_[2]));
    GPU_CHECK(cudaFree(rpn_buffers_[3]));
    GPU_CHECK(cudaFree(rpn_buffers_[4]));
    GPU_CHECK(cudaFree(rpn_buffers_[5]));
    GPU_CHECK(cudaFree(rpn_buffers_[6]));

    GPU_CHECK(cudaFree(dev_score_indexs_));
    GPU_CHECK(cudaFreeHost(host_keep_data_));
    GPU_CHECK(cudaFreeHost(host_boxes_));
    GPU_CHECK(cudaFreeHost(host_label_));
    GPU_CHECK(cudaFreeHost(host_score_indexs_));
    GPU_CHECK(cudaFreeHost(mask_cpu));
    GPU_CHECK(cudaFreeHost(remv_cpu));
}

void CenterPoint::Init(YAML::Node &config) {
//    reproduce_result_mode = true;
//    num_threads = 64;
//    max_num_pillars = 32000;
//    max_num_points_per_pillar = 20;
//    pfe_output_size = 64;
//    pillar_x_size = 0.16, pillar_y_size = 0.16, pillar_z_size = 0.4;
//    grid_x_size = 468, grid_y_size = 468, grid_z_size = 1;
//
//    num_inds_for_scan = 512;
//    min_x_range = -74.88f, min_y_range = -74.88f, min_z_range = -2.0f;
//    max_x_range = 74.88f, max_y_range = 74.88f, max_z_range = -4.0f;
//    num_box_corners = 4;

//    preprocess_points_.reset(new PreprocessPoints(max_num_pillars, max_num_points_per_pillar, grid_x_size,
//            grid_y_size, grid_z_size, pillar_x_size, pillar_y_size,
//            pillar_z_size, min_x_range, min_y_range, min_z_range,
//            num_inds_for_scan, num_box_corners));


    pfe_onnx_file = "/home/ricky/waytousLIB/lidar_detect/CenterPointTensorRT-master/pfe_baseline32000.onnx";
    rpn_onnx_file = "/home/ricky/waytousLIB/lidar_detect/CenterPointTensorRT-master/rpn_baseline.onnx";




    preprocess_points_.reset(new PreProcess_(64));
    scatter_cuda_.reset(new ScatterCudaV2(PFE_OUTPUT_DIM, PFE_OUTPUT_DIM, BEV_W, BEV_H));
    postprocess_cuda_.reset(new PostProcessAnchorFree());
    initEngine();
    deviceMemoryMalloc();

}

void CenterPoint::deviceMemoryMalloc() {

    /**
     *@brief : Create and Init Variables for points
     **/
    //GPU_CHECK(cudaMalloc((void**)&dev_points, MAX_POINTS * POINT_DIM * sizeof(float)));

    GPU_CHECK(cudaMalloc((void**)&deviceIndices,MAX_PILLARS * sizeof(int)));

    GPU_CHECK(cudaMalloc((void**)&devicePillars, MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float)));

    /**
     * @brief : Create and Init Variables for PreProcess
     **/
    GPU_CHECK(cudaMalloc((void**)& _PBEVIdxs, MAX_POINTS * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)& _PPointNumAssigned, MAX_POINTS * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)& _PMask, MAX_POINTS * sizeof(bool)));
    GPU_CHECK(cudaMalloc((void**)& _BEVVoxelIdx, BEV_H * BEV_W * sizeof(int)));

    GPU_CHECK(cudaMalloc((void**)&_VPointSum, MAX_PILLARS * 3 *sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&_VRange, MAX_PILLARS * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)&_VPointNum, MAX_PILLARS * sizeof(int)));

    GPU_CHECK(cudaMalloc(&pfe_buffers_[0], MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[1], MAX_PILLARS * PFE_OUTPUT_DIM * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_scattered_feature_, PFE_OUTPUT_DIM * BEV_W * BEV_H * sizeof(float)));

    GPU_CHECK(cudaMalloc(&rpn_buffers_[0], PFE_OUTPUT_DIM * BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[1], 2*BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[2], BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[3], 2 * BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[4], 3 * BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[5], BEV_W * BEV_H * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[6], BEV_W * BEV_H * sizeof(float)));

    GPU_CHECK(cudaMalloc((void**)&dev_score_indexs_, OUTPUT_W * OUTPUT_H * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_score_indexs_, -1 , OUTPUT_W * OUTPUT_H * sizeof(int)));

    GPU_CHECK(cudaMallocHost((void**)& mask_cpu, INPUT_NMS_MAX_SIZE * DIVUP (INPUT_NMS_MAX_SIZE ,THREADS_PER_BLOCK_NMS) * sizeof(unsigned long long)));
    GPU_CHECK(cudaMemset(mask_cpu, 0 ,  INPUT_NMS_MAX_SIZE * DIVUP (INPUT_NMS_MAX_SIZE ,THREADS_PER_BLOCK_NMS) * sizeof(unsigned long long)));

    GPU_CHECK(cudaMallocHost((void**)& remv_cpu, THREADS_PER_BLOCK_NMS * sizeof(unsigned long long)));
    GPU_CHECK(cudaMemset(remv_cpu, 0 ,  THREADS_PER_BLOCK_NMS  * sizeof(unsigned long long)));

    GPU_CHECK(cudaMallocHost((void**)&host_score_indexs_, OUTPUT_W * OUTPUT_H  * sizeof(int)));
    GPU_CHECK(cudaMemset(host_score_indexs_, -1, OUTPUT_W * OUTPUT_H  * sizeof(int)));

    GPU_CHECK(cudaMallocHost((void**)&host_keep_data_, INPUT_NMS_MAX_SIZE * sizeof(long)));
    GPU_CHECK(cudaMemset(host_keep_data_, -1, INPUT_NMS_MAX_SIZE * sizeof(long)));

    GPU_CHECK(cudaMallocHost((void**)&host_boxes_, OUTPUT_NMS_MAX_SIZE * 9 * sizeof(float)));
    GPU_CHECK(cudaMemset(host_boxes_, 0 ,  OUTPUT_NMS_MAX_SIZE * 9 * sizeof(float)));

    GPU_CHECK(cudaMallocHost((void**)&host_label_, OUTPUT_NMS_MAX_SIZE * sizeof(int)));
    GPU_CHECK(cudaMemset(host_label_, -1, OUTPUT_NMS_MAX_SIZE * sizeof(int)));


}

void CenterPoint::PreProcess(const float *in_points_array, const int in_num_points) {


    float* dev_points;

    //GPU_CHECK(cudaMemset(dev_points,0,in_num_points * POINT_DIM * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_points, in_num_points * POINT_DIM * sizeof(float)));
    GPU_CHECK(cudaMemcpy(dev_points, in_points_array, in_num_points * POINT_DIM * sizeof(float), cudaMemcpyHostToDevice));

   // GPU_CHECK(cudaMemset(dev_points,0, MAX_POINTS * POINT_DIM * sizeof(float)));
    GPU_CHECK(cudaMemset(deviceIndices,0,MAX_PILLARS * sizeof(int)));
    GPU_CHECK(cudaMemset(devicePillars,0,MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float)));

    GPU_CHECK(cudaMemset(_PBEVIdxs, 0, MAX_POINTS * sizeof(int)));
    GPU_CHECK(cudaMemset(_PPointNumAssigned, 0, MAX_POINTS * sizeof(int)));
    GPU_CHECK(cudaMemset(_PMask, 0, MAX_POINTS * sizeof(bool)));
    GPU_CHECK(cudaMemset(_BEVVoxelIdx, 0, BEV_H * BEV_W * sizeof(int)));

    GPU_CHECK(cudaMemset(_VPointNum,0, MAX_PILLARS * sizeof(int)));
    GPU_CHECK(cudaMemset(_VRange,0, MAX_PILLARS * sizeof(int)));
    GPU_CHECK(cudaMemset(_VPointSum, 0, MAX_PILLARS * 3 * sizeof(float)));


    preprocess_points_->preprocessGPU(dev_points, devicePillars,deviceIndices,
                                     _PMask, _PBEVIdxs,  _PPointNumAssigned,  _BEVVoxelIdx, _VPointSum,  _VRange,  _VPointNum,
                                      in_num_points, POINT_DIM);


    GPU_CHECK(cudaFree(dev_points));

    gLogInfo << "<<preprocessGPU>> "   << std::endl;

}

void CenterPoint::Detect(const float *in_points_array, const int in_num_points, std::vector<float> &out_detections,  std::vector<Box3D>& predResult) {
    // Create and Init Variables for device

    auto t_start = std::chrono::high_resolution_clock::now();

    PreProcess(in_points_array, in_num_points);

    //float test[MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM];
    //GPU_CHECK(cudaMemcpy(test, devicePillars, MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float), cudaMemcpyDeviceToHost));
   // gLogInfo << "TEST RESULTS: " << test[100] << std::endl;

    auto t_end1 = std::chrono::high_resolution_clock::now();
    float total1 = std::chrono::duration<float, std::milli>(t_end1- t_start).count();
    gLogInfo <<  "lidar PreProcess exec time: " << total1 << " ms." << std::endl;

    GPU_CHECK(cudaStreamCreate(&stream_));

    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[0], devicePillars, MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    auto flag = pfe_context_->enqueue(1, pfe_buffers_, stream_, nullptr);

    gLogInfo << "<<<<<<<<<<<<<<<<<<pfe FLOAG>>>>>>>>>>>>>>>>>>>>>:" << flag << std::endl;

    auto t_end2 = std::chrono::high_resolution_clock::now();
    float total2 = std::chrono::duration<float, std::milli>(t_end2- t_end1).count();
    gLogInfo <<  "lidar pfe exec time: " << total2 << " ms." << std::endl;


    GPU_CHECK(cudaMemset(dev_scattered_feature_, 0, PFE_OUTPUT_DIM * BEV_W * BEV_H * sizeof(float)));

    scatter_cuda_->doScatterCuda(MAX_PILLARS, deviceIndices, (float*)pfe_buffers_[1], dev_scattered_feature_);

    auto t_end3 = std::chrono::high_resolution_clock::now();
    float total3 = std::chrono::duration<float, std::milli>(t_end3- t_end2).count();
    gLogInfo <<  "lidar scatter exec time: " << total3 << " ms." << std::endl;

    GPU_CHECK(cudaMemcpyAsync(rpn_buffers_[0], dev_scattered_feature_, PFE_OUTPUT_DIM * BEV_W * BEV_H * sizeof(float),
                              cudaMemcpyDeviceToDevice, stream_));

    auto flag_ = rpn_context_->enqueue(1, rpn_buffers_, stream_, nullptr);

    gLogInfo << "<<<<<<<<<<<<<<<<<<rpn FLOAG>>>>>>>>>>>>>>>>>>>>>:" << flag_ << std::endl;

    output_result_.clear();

//    float test[2*BEV_W * BEV_H ];
//    GPU_CHECK(cudaMemcpy(test, rpn_buffers_[1], 2*BEV_W * BEV_H * sizeof(float),cudaMemcpyDeviceToHost));
//    for(int i = 0; i< BEV_W * BEV_H + 100; i++){
//        std::cout << " rpn_buffers_" << i << ": " << test[i] << std::endl;
//    }
    //gLogInfo << "test result is : " << test[5000] << std::endl;




    postprocess_cuda_->postprocessGPU((float*)rpn_buffers_[1],
                                      (float*)rpn_buffers_[2],
                                      (float*)rpn_buffers_[3],
                                      (float*)rpn_buffers_[4],
                                      (float*)rpn_buffers_[5],
                                      (int32_t*)rpn_buffers_[6],
                                      output_result_,
                                      dev_score_indexs_,
                                      mask_cpu,
                                      remv_cpu,
                                      host_score_indexs_,
                                      host_keep_data_,
                                      host_boxes_,
                                      host_label_);

    predResult = output_result_;

    auto t_end4 = std::chrono::high_resolution_clock::now();
    float total4 = std::chrono::duration<float, std::milli>(t_end4- t_end3).count();
    gLogInfo <<  "lidar rpn exec time: " << total4 << " ms." << std::endl;

    gLogInfo << "___________________________" << std::endl;


    cudaStreamDestroy(stream_);



}

void CenterPoint::initEngine() {
    // create a TensorRT model from the onnx model and serialize it to a stream
    nvinfer1::IHostMemory* pfe_trt_model_stream{ nullptr };
    nvinfer1::IHostMemory* rpn_trt_model_stream{ nullptr };
    onnxToTRTModel(pfe_onnx_file, pfe_trt_model_stream);
    onnxToTRTModel(rpn_onnx_file, rpn_trt_model_stream);

    //if (pfe_trt_model_stream == nullptr)
    if (pfe_trt_model_stream == nullptr || rpn_trt_model_stream == nullptr)
    {//use std:cerr instead of ROS_ERROR because want to keep this fille ros-agnostics
        std::cerr<< "Failed to load ONNX file " << std::endl;
    }

    // deserialize the engine
    pfe_runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    rpn_runtime_ = nvinfer1::createInferRuntime(trt_logger_);

    //if (pfe_runtime_ == nullptr)
    if (pfe_runtime_ == nullptr || rpn_runtime_ == nullptr)
    {
        std::cerr<<"Failed to create TensorRT Runtime object."<<std::endl;
    }

    pfe_engine_ =
            pfe_runtime_->deserializeCudaEngine(pfe_trt_model_stream->data(), pfe_trt_model_stream->size(), nullptr);
    rpn_engine_ =
            rpn_runtime_->deserializeCudaEngine(rpn_trt_model_stream->data(), rpn_trt_model_stream->size(), nullptr);
    //if (pfe_engine_ == nullptr)


//    int input_index = rpn_engine_->getBindingIndex("input.1");
//    int output_index1 = rpn_engine_->getBindingIndex("246");
//    int output_index2 = rpn_engine_->getBindingIndex("250");
//    int output_index3 = rpn_engine_->getBindingIndex("258");
//    int output_index4 = rpn_engine_->getBindingIndex("264");
//    int output_index5 = rpn_engine_->getBindingIndex("265");
//    int output_index6 = rpn_engine_->getBindingIndex("266");
//    gLogInfo <<" ###################### input_index: " << input_index << std::endl;
//    gLogInfo <<" ###################### output_index1: " << output_index1 << std::endl;
//    gLogInfo <<" ###################### output_index2: " << output_index2 << std::endl;
//    gLogInfo <<" ###################### output_index3: " << output_index3 << std::endl;
//    gLogInfo <<" ###################### output_index4: " << output_index4 << std::endl;
//    gLogInfo <<" ###################### output_index5: " << output_index5 << std::endl;
//    gLogInfo <<" ###################### output_index6: " << output_index6 << std::endl;






    if (pfe_engine_ == nullptr || rpn_engine_ == nullptr)
    {
        std::cerr << "Failed to create TensorRT Engine." << std::endl;
    }

    pfe_trt_model_stream->destroy();
    rpn_trt_model_stream->destroy();
    pfe_context_ = pfe_engine_->createExecutionContext();
    rpn_context_ = rpn_engine_->createExecutionContext();
      if (pfe_context_ == nullptr)
    if (pfe_context_ == nullptr || rpn_context_ == nullptr)
    {
        std::cerr << "Failed to create TensorRT Execution Context." << std::endl;;
    }
}
void CenterPoint::onnxToTRTModel(const std::string &model_file, nvinfer1::IHostMemory *&trt_model_stream) {
    int verbosity = (int)nvinfer1::ILogger::Severity::kWARNING;

    // create the builder
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(trt_logger_);
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    //nvinfer1::INetworkDefinition* network = builder->createNetwork();

    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicitBatch);

    auto parser = nvonnxparser::createParser(*network, trt_logger_);

    if (!parser->parseFromFile(model_file.c_str(), verbosity))
    {
        std::string msg("failed to parse onnx file");
        trt_logger_.log(nvinfer1::ILogger::Severity::kERROR, msg.c_str());
        exit(EXIT_FAILURE);
    }

    // Build the engine
    builder->setMaxBatchSize(1);
    //builder->setMaxWorkspaceSize(1 << 20);
    config->setMaxWorkspaceSize(1 << 30);
    config->setFlag(BuilderFlag::kFP16);


    //nvinfer1::ICudaEngine* engine = builder->buildCudaEngine(*network);
    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network,*config);


    parser->destroy();

    // serialize the engine, then close everything down
    trt_model_stream = engine->serialize();
    engine->destroy();
    network->destroy();
    builder->destroy();
}