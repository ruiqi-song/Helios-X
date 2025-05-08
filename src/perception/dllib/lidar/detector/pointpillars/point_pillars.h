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

/**
* @file point_pillars.h
* @brief Algorithm for PointPillars
* @author Kosuke Murakami, Luca Fancellu
* @date 2020/09/04
*/

#pragma once

// headers in STL
#include <vector>
#include <iomanip>
#include <string>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>


  // headers in TensorRT
#include "NvInfer.h"
#include "NvOnnxParser.h"


#include "base/lidar_base_detector.h"

// headers in local files
#include "lidar/detector/common/common.h"
#include "lidar/detector/common/preprocess/preprocess_anchor_based.h"
#include "lidar/detector/common/preprocess/preprocess_anchor_based_cuda.h"
#include "lidar/detector/common/anchor/anchor_mask_cuda.h"
#include "lidar/detector/common/scatter/scatter_cuda.h"
#include "lidar/detector/common/postprocess/postprocess_anchor_based.h"

#include "register/register.h"
#include "monitor/log.h"
#include "config/config.h"
#include "inference/tensorrt/tensorrt_net.h"

class PointPillars : public LidarBaseDetector
{
public:

    PointPillars();

    ~PointPillars();

    void Init(YAML::Node& config) override ;
    void Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,  std::vector<Box3D>& predResult) override ;

private:

    void deviceMemoryMalloc();
    void initAnchors();
    void initEngine();
    void initEngine_();
    void generateAnchors(float* anchors_px_, float* anchors_py_, float* anchors_pz_, float* anchors_dx_,
                         float* anchors_dy_, float* anchors_dz_, float* anchors_ro_);


    void onnxToTRTModel(const std::string& model_file, nvinfer1::IHostMemory*& trt_model_stream);

    void preprocess(const float* in_points_array, const int in_num_points);
    void preprocessCPU(const float* in_points_array, const int in_num_points);

    void preprocessGPU(const float* in_points_array, const int in_num_points);
    void convertAnchors2BoxAnchors(float* anchors_px_, float* anchors_py_, float* anchors_dx_, float* anchors_dy_,
                                   float* box_anchors_min_x_, float* box_anchors_min_y_, float* box_anchors_max_x_,
                                   float* box_anchors_max_y_);
    void putAnchorsInDeviceMemory();

    std::unique_ptr<ConfigParser> config_;

    TRTLogger trt_logger_;
    nvinfer1::IExecutionContext* pfe_context_;
    nvinfer1::IExecutionContext* rpn_context_;
    nvinfer1::IRuntime* pfe_runtime_;
    nvinfer1::IRuntime* rpn_runtime_;
    nvinfer1::ICudaEngine* pfe_engine_;
    nvinfer1::ICudaEngine* rpn_engine_;

    std::unique_ptr<PreprocessPoints> preprocess_points_ptr_;
    std::unique_ptr<PreprocessPointsCuda> preprocess_points_cuda_ptr_;
    std::unique_ptr<AnchorMaskCuda> anchor_mask_cuda_ptr_;
    std::unique_ptr<ScatterCuda> scatter_cuda_ptr_;
    std::unique_ptr<PostprocessCuda> postprocess_cuda_ptr_;


    std::unique_ptr<TensorRTNet> tensorrt_net_ptr_;

    int num_classes;
    const bool reproduce_result_mode;
    float score_threshold;
    float nms_overlap_threshold;
    std::string pfe_onnx_file, rpn_onnx_file;
    int max_num_pillars = 12000;
    int max_num_points_per_pillar;
    int pfe_output_size;
    int grid_x_size, grid_y_size, grid_z_size;
    int rpn_input_size;
    int num_anchor_x_inds, num_anchor_y_inds, num_anchor_r_inds;
    int num_anchor;
    int rpn_box_output_size, rpn_cls_output_size, rpn_dir_output_size;
    float pillar_x_size, pillar_y_size, pillar_z_size;
    float min_x_range, min_y_range, min_z_range;
    float max_x_range, max_y_range, max_z_range;
    int batch_size;
    int num_inds_for_scan;
    int num_threads;
    float sensor_height;
    float anchor_dx_size, anchor_dy_size, anchor_dz_size;
    int num_box_corners;
    int num_output_box_feature;

    // end initializer list
    int host_pillar_count_[1];
    float* anchors_px_, * anchors_py_, * anchors_pz_;
    float* anchors_dx_, * anchors_dy_, * anchors_dz_;
    float* anchors_ro_;

    float* box_anchors_min_x_, * box_anchors_min_y_;
    float* box_anchors_max_x_, * box_anchors_max_y_;
    //float* box_anchors_max_y_;

  // cuda malloc
    float* dev_pillar_x_in_coors_, * dev_pillar_y_in_coors_, * dev_pillar_z_in_coors_, * dev_pillar_i_in_coors_;
    int* dev_pillar_count_histo_;

    int* dev_x_coors_, * dev_y_coors_;
    float* dev_num_points_per_pillar_;
    int* dev_sparse_pillar_map_;
    int* dev_cumsum_along_x_, * dev_cumsum_along_y_;


    float* dev_pillar_x_, * dev_pillar_y_, * dev_pillar_z_, * dev_pillar_i_;

    float* dev_x_coors_for_sub_shaped_;
    float* dev_y_coors_for_sub_shaped_;
    float* dev_pillar_feature_mask_;

    float* dev_box_anchors_min_x_, *dev_box_anchors_min_y_;

    float* dev_box_anchors_max_x_, *dev_box_anchors_max_y_;
    int* dev_anchor_mask_;

    void* pfe_buffers_[9];
    void* rpn_buffers_[4];

    cudaStream_t stream_;

    float* dev_scattered_feature_;

    float* dev_anchors_px_, *dev_anchors_py_, *dev_anchors_pz_;
    float* dev_anchors_dx_, *dev_anchors_dy_, *dev_anchors_dz_;
    float* dev_anchors_ro_;
    float* dev_filtered_box_;
    float* dev_filtered_score_;
    int* dev_filtered_cls_;
    int* dev_filtered_dir_;
    float* dev_box_for_nms_;
    int* dev_filter_count_;

    std::vector<Box3D> output_result_;

};

REGISTER_LIDAR_DETECTOR(PointPillars);
