//
// Created by ricky on 2022/4/14.
//

/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @author Kosuke Murakami
 * @date 2019/02/26
 */

/**
* @author Yan haixu
* Contact: just github.com/hova88
* @date 2021/04/30
*/


#pragma once

// headers in STL
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
// headers in TensorRT
#include "NvInfer.h"
#include "NvOnnxParser.h"

// headers in local files
// #include "params.h"
#include <yaml-cpp/yaml.h>
#include "register/register.h"
#include "monitor/log.h"
#include "config/config.h"
#include "base/lidar_base_detector.h"
#include "lidar/detector/common/common.h"
#include <yaml-cpp/yaml.h>
#include "lidar/detector/common/preprocess/preprocess_multi_heads.h"
#include "lidar/detector/common/scatter//scatter_cuda.h"
#include "lidar/detector/common/postprocess//postprocess_multi_heads.h"
#include "inference/tensorrt/tensorrt_net.h"

using namespace std;

class PointPillarsNuScene : public LidarBaseDetector{
private:
    // initialize in initializer list
    float score_threshold_;
    float nms_overlap_threshold_;
    bool use_onnx_;
    std::string pfe_file_;
    std::string backbone_file_;
    std::string pp_config_;
    // end initializer list
    // voxel size
    float kPillarXSize;
    float kPillarYSize;
    float kPillarZSize;
    // point cloud range
    float kMinXRange;
    float kMinYRange;
    float kMinZRange;
    float kMaxXRange;
    float kMaxYRange;
    float kMaxZRange;
    // hyper parameters
    int kNumClass;
    int kMaxNumPillars;
    int kMaxNumPointsPerPillar;
    int kNumPointFeature;
    int kNumGatherPointFeature = 11;
    int kGridXSize;
    int kGridYSize;
    int kGridZSize;
    int kNumAnchorXinds;
    int kNumAnchorYinds;
    int kRpnInputSize;
    int kNumAnchor;
    int kNumInputBoxFeature;
    int kNumOutputBoxFeature;
    int kRpnBoxOutputSize;
    int kRpnClsOutputSize;
    int kRpnDirOutputSize;
    int kBatchSize;
    int kNumIndsForScan;
    int kNumThreads;
    // if you change kNumThreads, need to modify NUM_THREADS_MACRO in
    // common.h
    int kNumBoxCorners;
    int kNmsPreMaxsize;
    int kNmsPostMaxsize;
    //params for initialize anchors
    //Adapt to OpenPCDet
    int kAnchorStrides;
    std::vector<string> kAnchorNames;
    std::vector<float> kAnchorDxSizes;
    std::vector<float> kAnchorDySizes;
    std::vector<float> kAnchorDzSizes;
    std::vector<float> kAnchorBottom;
    std::vector<std::vector<int>> kMultiheadLabelMapping;
    int kNumAnchorPerCls;
    int host_pillar_count_[1];

    int* dev_x_coors_;
    int* dev_y_coors_;
    float* dev_num_points_per_pillar_;
    int* dev_sparse_pillar_map_;
    int* dev_cumsum_along_x_;
    int* dev_cumsum_along_y_;

    float* dev_pillar_point_feature_;
    float* dev_pillar_coors_;
    float* dev_points_mean_;

    float* dev_pfe_gather_feature_;
    void* pfe_buffers_[2];
    //variable for doPostprocessCudaMultiHead
    void* rpn_buffers_[8];

    std::vector<float*> rpn_box_output_;
    std::vector<float*> rpn_cls_output_;

    float* dev_scattered_feature_;

    float* host_box_;
    float* host_score_;
    int*   host_filtered_count_;

    // float* dev_filtered_box_;
    // float* dev_filtered_score_;
    // int*   dev_filtered_label_;
    // int*   dev_filtered_dir_;
    // float* dev_box_for_nms_;
    // int*   dev_filter_count_;

    std::unique_ptr<PreprocessPointsMultiHeadsCuda> preprocess_points_cuda_ptr_;
    std::unique_ptr<ScatterMultiHeadsCuda> scatter_cuda_ptr_;
    std::unique_ptr<PostprocessMultiHeadsCuda> postprocess_cuda_ptr_;

    TRTLogger g_logger_;
    nvinfer1::ICudaEngine* pfe_engine_;
    nvinfer1::ICudaEngine* backbone_engine_;
    nvinfer1::IExecutionContext* pfe_context_;
    nvinfer1::IExecutionContext* backbone_context_;

    /**
     * @brief Memory allocation for device memory
     * @details Called in the constructor
     */
    void DeviceMemoryMalloc();

    /**
     * @brief Memory set to 0 for device memory
     * @details Called in the DoInference
     */
    void SetDeviceMemoryToZero();

    /**
     * @brief Initializing paraments from pointpillars.yaml
     * @details Called in the constructor
     */
    void InitParams();
    /**
     * @brief Initializing TensorRT instances
     * @param[in] usr_onnx_ if true, parse ONNX
     * @details Called in the constructor
     */
    void InitTRT(const bool use_onnx);
    /**
     * @brief Convert ONNX to TensorRT model
     * @param[in] model_file ONNX model file path
     * @param[out] engine_ptr TensorRT model engine made out of ONNX model
     * @details Load ONNX model, and convert it to TensorRT model
     */
    void OnnxToTRTModel(const std::string& model_file,
                        nvinfer1::ICudaEngine** engine_ptr);

    /**
     * @brief Convert Engine to TensorRT model
     * @param[in] model_file Engine(TensorRT) model file path
     * @param[out] engine_ptr TensorRT model engine made
     * @details Load Engine model, and convert it to TensorRT model
     */
    void EngineToTRTModel(const std::string &engine_file ,
                          nvinfer1::ICudaEngine** engine_ptr) ;

    /**
     * @brief Preproces points
     * @param[in] in_points_array Point cloud array
     * @param[in] in_num_points Number of points
     * @details Call CPU or GPU preprocess
     */
    void Preprocess(const float* in_points_array, const int in_num_points);

public:
    PointPillarsNuScene();
    ~PointPillarsNuScene();


    void Init(YAML::Node& config) override ;
    void Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,
                std::vector<Box3D>& predResult) override ;
    /**
     * @brief Constructor
     * @param[in] score_threshold Score threshold for filtering output
     * @param[in] nms_overlap_threshold IOU threshold for NMS
     * @param[in] use_onnx if true,using onnx file ,else using engine file
     * @param[in] pfe_file Pillar Feature Extractor ONNX file path
     * @param[in] rpn_file Region Proposal Network ONNX file path
     * @details Variables could be changed through point_pillars_detection
     */


    /**
     * @brief Call PointPillars for the inference
     * @param[in] in_points_array Point cloud array
     * @param[in] in_num_points Number of points
     * @param[out] out_detections Network output bounding box
     * @param[out] out_labels Network output object's label
     * @details This is an interface for the algorithm
     */

};
REGISTER_LIDAR_DETECTOR(PointPillarsNuScene);


