//
// Created by ricky on 2022/3/24.
//

#pragma once

#include "NvInfer.h"
#include "NvOnnxParser.h"

#include "yaml-cpp/yaml.h"

//#include "config/config.h"
#include "register/register.h"

#include "inference/tensorrt/tensorrt_net.h"

#include "base/lidar_base_detector.h"

#include "lidar/detector/common/common.h"
#include "lidar/detector/common/preprocess/preprocess_anchor_based.h"
#include "lidar/detector/common/preprocess/preprocess_anchor_based_cuda.h"
#include "lidar/detector/common/scatter/scatter_cuda.h"
#include "lidar/detector/common/postprocess/postprocess_anchor_based.h"

#include "config.h"
#include "lidar/detector/common/preprocess/preprocess_center_based.h"
#include "lidar/detector/common/postprocess/postprocess_center_based.h"

class CenterPoint : public LidarBaseDetector{
public:
    CenterPoint();
    ~CenterPoint();
    void Init(YAML::Node& config) override ;
    void Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,  std::vector<Box3D>& predResult) override ;

private:
    void initEngine();
    void onnxToTRTModel(const std::string& model_file, nvinfer1::IHostMemory*& trt_model_stream);


    void PreProcess(const float* in_points_array, const int in_num_points);
    void deviceMemoryMalloc();


    //std::unique_ptr<ConfigParser> config_;

    TRTLogger trt_logger_;
    cudaStream_t stream_;
    nvinfer1::IExecutionContext* pfe_context_;
    nvinfer1::IExecutionContext* rpn_context_;
    nvinfer1::IRuntime* pfe_runtime_;
    nvinfer1::IRuntime* rpn_runtime_;
    nvinfer1::ICudaEngine* pfe_engine_;
    nvinfer1::ICudaEngine* rpn_engine_;

    std::unique_ptr<PreProcess_> preprocess_points_;


    std::string pfe_onnx_file, rpn_onnx_file;
    //std::unique_ptr<PreprocessPointsCuda> preprocess_points_cuda_;
    std::unique_ptr<ScatterCudaV2> scatter_cuda_;
    std::unique_ptr<PostProcessAnchorFree> postprocess_cuda_;


//    bool reproduce_result_mode = true;
//    int num_threads;
//    int max_num_pillars;
//    int max_num_points_per_pillar;
//    int pfe_output_size;
//    float pillar_x_size, pillar_y_size, pillar_z_size;
//    int grid_x_size, grid_y_size, grid_z_size;
//
//    int num_inds_for_scan;
//    float min_x_range, min_y_range, min_z_range;
//    float max_x_range, max_y_range, max_z_range;
//    int num_box_corners;


//    int* dev_pillar_count_histo_;
//    int* dev_x_coors_, * dev_y_coors_;
//    float* dev_num_points_per_pillar_;
//    int* dev_sparse_pillar_map_;
//    float* dev_pillar_x_, * dev_pillar_y_, * dev_pillar_z_, * dev_pillar_i_;
//    int* dev_anchor_mask_;
//    float* dev_x_coors_for_sub_shaped_;
//    float* dev_y_coors_for_sub_shaped_;

    float* dev_points;
    int* deviceIndices;
    float* devicePillars;

    void* pfe_buffers_[2];
    void* rpn_buffers_[7];

    float* dev_scattered_feature_;


    // device pointers for preprocess
    int* _PBEVIdxs;
    int* _PPointNumAssigned;
    bool* _PMask;
    int* _BEVVoxelIdx; // H * W
    float* _VPointSum;
    int* _VRange;
    int* _VPointNum;

    // host  variables for post process
    long* host_keep_data_;
    float* host_boxes_;
    int* host_label_;
    int* host_score_indexs_;
    unsigned long long* mask_cpu;
    unsigned long long* remv_cpu;

    int* dev_score_indexs_;

    std::vector<Box3D> output_result_;

};

REGISTER_LIDAR_DETECTOR(CenterPoint);
