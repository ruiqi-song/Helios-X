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

// headers in STL
#include <chrono>
#include <iostream>

// headers in local files
#include "lidar/detector/pointpillars/point_pillars.h"
// clang-format off
PointPillars::PointPillars()
  : reproduce_result_mode(true)
//  , score_threshold(0.5)
//  , nms_overlap_threshold(0.5)
//  , pfe_onnx_file("/home/ricky/lidar/pointpillars/src/lidar_point_pillars/pfe.onnx")
//  , rpn_onnx_file("/home/ricky/lidar/pointpillars/src/lidar_point_pillars/rpn.onnx")
//  , max_num_pillars(12000)
//  , max_num_points_per_pillar(100)
//  , pfe_output_size(max_num_pillars * 64)
//  , grid_x_size(432)
//  , grid_y_size(496)
//  , grid_z_size(1)
//  , rpn_input_size(64 * grid_x_size * grid_y_size)
//  , num_anchor_x_inds(grid_x_size * 0.5)
//  , num_anchor_y_inds(grid_y_size * 0.5)
//  , num_anchor_r_inds(2)
//  , num_anchor(num_anchor_x_inds * num_anchor_y_inds * num_anchor_r_inds)
//  , rpn_box_output_size(num_anchor * 7)
//  , rpn_cls_output_size(num_anchor)
//  , rpn_dir_output_size(num_anchor * 2)
//  , pillar_x_size(0.16f)
//  , pillar_y_size(0.16f)
//  , pillar_z_size(4.0f)
//  , min_x_range(0.0f)
//  , min_y_range(-39.68f)
//  , min_z_range(-3.0f)
//  , max_x_range(69.12f)
//  , max_y_range(39.68f)
//  , max_z_range(1)
//  , batch_size(1)
//  , num_inds_for_scan(512)
//  , num_threads(64) // if you change NUM_THREADS_, need to modify NUM_THREADS_MACRO in common.h
//  , sensor_height(1.73f)
//  , anchor_dx_size(1.6f)
//  , anchor_dy_size(3.9f)
//  , anchor_dz_size(1.56f)
//  , num_box_corners(4)
//  , num_output_box_feature(7)
{
//  if (reproduce_result_mode)
//  {
//    preprocess_points_ptr_.reset(new PreprocessPoints(max_num_pillars, max_num_points_per_pillar, grid_x_size,
//                                                      grid_y_size, grid_z_size, pillar_x_size, pillar_y_size,
//                                                      pillar_z_size, min_x_range, min_y_range, min_z_range,
//                                                      num_inds_for_scan, num_box_corners));
//  }
//  else
//  {
//    preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
//            num_threads, max_num_pillars, max_num_points_per_pillar, num_inds_for_scan, grid_x_size, grid_y_size,
//        grid_z_size, pillar_x_size, pillar_y_size, pillar_z_size, min_x_range, min_y_range, min_z_range, num_box_corners));
//  }
//
//  anchor_mask_cuda_ptr_.reset(new AnchorMaskCuda(num_inds_for_scan, num_anchor_x_inds, num_anchor_y_inds,
//                                                 num_anchor_r_inds, min_x_range, min_y_range, pillar_x_size,
//                                                 pillar_y_size, grid_x_size, grid_y_size));
//
//  scatter_cuda_ptr_.reset(new ScatterCuda(num_threads, max_num_pillars, grid_x_size, grid_y_size));
//
//  const float FLOAT_MIN = std::numeric_limits<float>::lowest();
//  const float FLOAT_MAX = std::numeric_limits<float>::max();
//  postprocess_cuda_ptr_.reset(new PostprocessCuda(FLOAT_MIN, FLOAT_MAX, num_anchor_x_inds, num_anchor_y_inds,
//                                                  num_anchor_r_inds, score_threshold, num_threads,
//                                                  nms_overlap_threshold, num_box_corners, num_output_box_feature));
//
//  deviceMemoryMalloc();
//  initEngine();
//  initAnchors();
}
// clang-format on



void PointPillars::Init(YAML::Node& config) {
    const std::string engine_path = config["component"]["detector"]["model"].as<std::string>();
    std::string conf_path = config["component"]["detector"]["config"].as<std::string>();
    config_.reset(new ConfigParser(conf_path));

    std::cout << "1" << std::endl;

    num_classes = std::stoi(config_->context["num_classes"]);

    score_threshold = std::stof(config_->context["score_threshold"]);
    nms_overlap_threshold = std::stof(config_->context["nms_overlap_threshold"]);
    pfe_onnx_file = config_->context["pfe_onnx_file"];
    rpn_onnx_file = config_->context["rpn_onnx_file"];

    max_num_pillars = std::stoi(config_->context["max_num_pillars"]);

    max_num_points_per_pillar = std::stoi(config_->context["max_num_points_per_pillar"]);
    pfe_output_size = max_num_pillars * 64;
    grid_x_size = std::stoi(config_->context["grid_x_size"]);
    grid_y_size = std::stoi(config_->context["grid_y_size"]);
    grid_z_size = std::stoi(config_->context["grid_z_size"]);
    rpn_input_size = 64 * grid_x_size * grid_y_size;
    num_anchor_x_inds = grid_x_size * 0.5;
    num_anchor_y_inds = grid_y_size * 0.5;
    num_anchor_r_inds = std::stoi(config_->context["num_anchor_r_inds"]);
    num_anchor = num_anchor_x_inds * num_anchor_y_inds * num_anchor_r_inds;
    rpn_box_output_size = num_anchor * 7;
    rpn_cls_output_size = num_anchor * 3;
    rpn_dir_output_size = num_anchor * 2;
    pillar_x_size = std::stof(config_->context["pillar_x_size"]);
    pillar_y_size = std::stof(config_->context["pillar_y_size"]);
    pillar_z_size = std::stof(config_->context["pillar_z_size"]);

    min_x_range = std::stof(config_->context["min_x_range"]);
    min_y_range = std::stof(config_->context["min_y_range"]);
    min_z_range = std::stof(config_->context["min_z_range"]);
    max_x_range = std::stof(config_->context["max_x_range"]);
    max_y_range = std::stof(config_->context["max_y_range"]);
    max_z_range = std::stof(config_->context["max_z_range"]);

    batch_size = std::stoi(config_->context["batch_size"]);
    num_inds_for_scan = std::stoi(config_->context["num_inds_for_scan"]);
    num_threads = std::stoi(config_->context["num_threads"]);
    sensor_height = std::stof(config_->context["sensor_height"]);
    anchor_dx_size = std::stof(config_->context["anchor_dx_size"]);
    anchor_dy_size = std::stof(config_->context["anchor_dy_size"]);
    anchor_dz_size = std::stof(config_->context["anchor_dz_size"]);
    num_box_corners = std::stoi(config_->context["num_box_corners"]);
    num_output_box_feature = std::stoi(config_->context["num_output_box_feature"]);

    bool isBuildEngine = false;

    tensorrt_net_ptr_.reset(new TensorRTNet(engine_path, isBuildEngine));


    if (reproduce_result_mode)
    {
        preprocess_points_ptr_.reset(new PreprocessPoints(max_num_pillars, max_num_points_per_pillar, grid_x_size,
                                                          grid_y_size, grid_z_size, pillar_x_size, pillar_y_size,
                                                          pillar_z_size, min_x_range, min_y_range, min_z_range,
                                                          num_inds_for_scan, num_box_corners));
    }
    else
    {
        preprocess_points_cuda_ptr_.reset(new PreprocessPointsCuda(
                num_threads, max_num_pillars, max_num_points_per_pillar, num_inds_for_scan, grid_x_size, grid_y_size,
                grid_z_size, pillar_x_size, pillar_y_size, pillar_z_size, min_x_range, min_y_range, min_z_range, num_box_corners));
    }

    anchor_mask_cuda_ptr_.reset(new AnchorMaskCuda(num_inds_for_scan, num_anchor_x_inds, num_anchor_y_inds,
                                                   num_anchor_r_inds, min_x_range, min_y_range, pillar_x_size,
                                                   pillar_y_size, grid_x_size, grid_y_size));

    scatter_cuda_ptr_.reset(new ScatterCuda(num_threads, max_num_pillars, grid_x_size, grid_y_size));

    const float FLOAT_MIN = std::numeric_limits<float>::lowest();
    const float FLOAT_MAX = std::numeric_limits<float>::max();
    postprocess_cuda_ptr_.reset(new PostprocessCuda(FLOAT_MIN, FLOAT_MAX, num_anchor_x_inds, num_anchor_y_inds,
                                                    num_anchor_r_inds, score_threshold, num_threads,
                                                    nms_overlap_threshold, num_box_corners, num_output_box_feature, num_classes));

    deviceMemoryMalloc();
    initEngine();
    //initEngine_();
    initAnchors();

}

PointPillars::~PointPillars()
{
    delete[] anchors_px_;
    delete[] anchors_py_;
    delete[] anchors_pz_;
    delete[] anchors_dx_;
    delete[] anchors_dy_;
    delete[] anchors_dz_;
    delete[] anchors_ro_;
    delete[] box_anchors_min_x_;
    delete[] box_anchors_min_y_;
    delete[] box_anchors_max_x_;
    delete[] box_anchors_max_y_;

    GPU_CHECK(cudaFree(dev_pillar_x_in_coors_));
    GPU_CHECK(cudaFree(dev_pillar_y_in_coors_));
    GPU_CHECK(cudaFree(dev_pillar_z_in_coors_));
    GPU_CHECK(cudaFree(dev_pillar_i_in_coors_));
    GPU_CHECK(cudaFree(dev_pillar_count_histo_));

    GPU_CHECK(cudaFree(dev_x_coors_));
    GPU_CHECK(cudaFree(dev_y_coors_));
    GPU_CHECK(cudaFree(dev_num_points_per_pillar_));
    GPU_CHECK(cudaFree(dev_sparse_pillar_map_));

    GPU_CHECK(cudaFree(dev_pillar_x_));
    GPU_CHECK(cudaFree(dev_pillar_y_));
    GPU_CHECK(cudaFree(dev_pillar_z_));
    GPU_CHECK(cudaFree(dev_pillar_i_));

    GPU_CHECK(cudaFree(dev_x_coors_for_sub_shaped_));
    GPU_CHECK(cudaFree(dev_y_coors_for_sub_shaped_));
    GPU_CHECK(cudaFree(dev_pillar_feature_mask_));

    GPU_CHECK(cudaFree(dev_cumsum_along_x_));
    GPU_CHECK(cudaFree(dev_cumsum_along_y_));

    GPU_CHECK(cudaFree(dev_box_anchors_min_x_));
    GPU_CHECK(cudaFree(dev_box_anchors_min_y_));
    GPU_CHECK(cudaFree(dev_box_anchors_max_x_));
    GPU_CHECK(cudaFree(dev_box_anchors_max_y_));
    GPU_CHECK(cudaFree(dev_anchor_mask_));

//#ifdef TVM_IMPLEMENTATION
//    GPU_CHECK(cudaFree(pfe_net_output));
//
//  GPU_CHECK(cudaFree(rpn_buffers_[0]));
//  GPU_CHECK(cudaFree(rpn_buffers_[1]));
//  GPU_CHECK(cudaFree(rpn_buffers_[2]));
//#else
    GPU_CHECK(cudaFree(pfe_buffers_[0]));
    GPU_CHECK(cudaFree(pfe_buffers_[1]));
    GPU_CHECK(cudaFree(pfe_buffers_[2]));
    GPU_CHECK(cudaFree(pfe_buffers_[3]));
    GPU_CHECK(cudaFree(pfe_buffers_[4]));
    GPU_CHECK(cudaFree(pfe_buffers_[5]));
    GPU_CHECK(cudaFree(pfe_buffers_[6]));
    GPU_CHECK(cudaFree(pfe_buffers_[7]));
    GPU_CHECK(cudaFree(pfe_buffers_[8]));

    GPU_CHECK(cudaFree(rpn_buffers_[0]));
    GPU_CHECK(cudaFree(rpn_buffers_[1]));
    GPU_CHECK(cudaFree(rpn_buffers_[2]));
    GPU_CHECK(cudaFree(rpn_buffers_[3]));
//#endif

    GPU_CHECK(cudaFree(dev_scattered_feature_));

    GPU_CHECK(cudaFree(dev_anchors_px_));
    GPU_CHECK(cudaFree(dev_anchors_py_));
    GPU_CHECK(cudaFree(dev_anchors_pz_));
    GPU_CHECK(cudaFree(dev_anchors_dx_));
    GPU_CHECK(cudaFree(dev_anchors_dy_));
    GPU_CHECK(cudaFree(dev_anchors_dz_));
    GPU_CHECK(cudaFree(dev_anchors_ro_));
    GPU_CHECK(cudaFree(dev_filtered_box_));
    GPU_CHECK(cudaFree(dev_filtered_score_));
    GPU_CHECK(cudaFree(dev_filtered_cls_));
    GPU_CHECK(cudaFree(dev_filtered_dir_));
    GPU_CHECK(cudaFree(dev_box_for_nms_));
    GPU_CHECK(cudaFree(dev_filter_count_));

#ifndef TVM_IMPLEMENTATION
    pfe_context_->destroy();
    rpn_context_->destroy();

    pfe_runtime_->destroy();
    rpn_runtime_->destroy();
    pfe_engine_->destroy();
    rpn_engine_->destroy();
#endif
}

void PointPillars::deviceMemoryMalloc()
{
    // clang-format off
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_x_in_coors_,grid_y_size * grid_x_size * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_y_in_coors_,grid_y_size * grid_x_size * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_z_in_coors_,grid_y_size * grid_x_size * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_i_in_coors_,grid_y_size * grid_x_size * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_count_histo_, grid_y_size * grid_x_size * sizeof(int)));
    // clang-format on

    GPU_CHECK(cudaMalloc((void**)&dev_x_coors_, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)&dev_y_coors_, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)&dev_num_points_per_pillar_, max_num_pillars * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_sparse_pillar_map_, num_inds_for_scan * num_inds_for_scan * sizeof(int)));

    GPU_CHECK(cudaMalloc((void**)&dev_pillar_x_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_y_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_z_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_i_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));

    // clang-format off
    GPU_CHECK(cudaMalloc((void**)&dev_x_coors_for_sub_shaped_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_y_coors_for_sub_shaped_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_pillar_feature_mask_, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    // clang-format on

    // cumsum kernel
    GPU_CHECK(cudaMalloc((void**)&dev_cumsum_along_x_, num_inds_for_scan * num_inds_for_scan * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)&dev_cumsum_along_y_, num_inds_for_scan * num_inds_for_scan * sizeof(int)));

    // for make anchor mask kernel
    GPU_CHECK(cudaMalloc((void**)&dev_box_anchors_min_x_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_box_anchors_min_y_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_box_anchors_max_x_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_box_anchors_max_y_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchor_mask_, num_anchor * sizeof(int)));

    // for trt inference
    // create GPU buffers and a stream

    GPU_CHECK(cudaMalloc(&pfe_buffers_[0], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[1], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[2], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[3], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[4], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[5], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[6], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[7], max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMalloc(&pfe_buffers_[8], pfe_output_size * sizeof(float)));

    GPU_CHECK(cudaMalloc(&rpn_buffers_[0], rpn_input_size * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[1], rpn_box_output_size * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[2], rpn_cls_output_size * sizeof(float)));
    GPU_CHECK(cudaMalloc(&rpn_buffers_[3], rpn_dir_output_size * sizeof(float)));


    // for scatter kernel
    GPU_CHECK(cudaMalloc((void**)&dev_scattered_feature_, num_threads * grid_y_size * grid_x_size * sizeof(float)));

    // for filter
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_px_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_py_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_pz_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_dx_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_dy_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_dz_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_anchors_ro_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_filtered_box_, num_anchor * num_output_box_feature * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_filtered_score_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_filtered_cls_, num_anchor * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_filtered_dir_, num_anchor * sizeof(int)));
    GPU_CHECK(cudaMalloc((void**)&dev_box_for_nms_, num_anchor * num_box_corners * sizeof(float)));
    GPU_CHECK(cudaMalloc((void**)&dev_filter_count_, sizeof(int)));
}

void PointPillars::initAnchors()
{
    // allocate memory for anchors
    anchors_px_ = new float[num_anchor];
    anchors_py_ = new float[num_anchor];
    anchors_pz_ = new float[num_anchor];
    anchors_dx_ = new float[num_anchor];
    anchors_dy_ = new float[num_anchor];
    anchors_dz_ = new float[num_anchor];
    anchors_ro_ = new float[num_anchor];
    box_anchors_min_x_ = new float[num_anchor];
    box_anchors_min_y_ = new float[num_anchor];
    box_anchors_max_x_ = new float[num_anchor];
    box_anchors_max_y_ = new float[num_anchor];
    // deallocate these memories in deconstructor

    generateAnchors(anchors_px_, anchors_py_, anchors_pz_, anchors_dx_, anchors_dy_, anchors_dz_, anchors_ro_);

    convertAnchors2BoxAnchors(anchors_px_, anchors_py_, anchors_dx_, anchors_dy_, box_anchors_min_x_, box_anchors_min_y_,
                              box_anchors_max_x_, box_anchors_max_y_);

    putAnchorsInDeviceMemory();
}

void PointPillars::generateAnchors(float* anchors_px_, float* anchors_py_, float* anchors_pz_, float* anchors_dx_,
                                   float* anchors_dy_, float* anchors_dz_, float* anchors_ro_)
{
    // zero clear
    for (size_t i = 0; i < num_anchor; i++)
    {
        anchors_px_[i] = 0;
        anchors_py_[i] = 0;
        anchors_pz_[i] = 0;
        anchors_dx_[i] = 0;
        anchors_dy_[i] = 0;
        anchors_dz_[i] = 0;
        anchors_ro_[i] = 0;
        box_anchors_min_x_[i] = 0;
        box_anchors_min_y_[i] = 0;
        box_anchors_max_x_[i] = 0;
        box_anchors_max_y_[i] = 0;
    }

    float x_stride = pillar_x_size * 2.0f;
    float y_stride = pillar_y_size * 2.0f;
    float x_offset = min_x_range + pillar_x_size;
    float y_offset = min_y_range + pillar_y_size;

    float anchor_x_count[num_anchor_x_inds] = { 0 };
    for (size_t i = 0; i < num_anchor_x_inds; i++)
    {
        anchor_x_count[i] = float(i) * x_stride + x_offset;
    }
    float anchor_y_count[num_anchor_y_inds] = { 0 };
    for (size_t i = 0; i < num_anchor_y_inds; i++)
    {
        anchor_y_count[i] = float(i) * y_stride + y_offset;
    }

    float anchor_r_count[num_anchor_r_inds] = { 0, M_PI / 2 };

    // np.meshgrid
    for (size_t y = 0; y < num_anchor_y_inds; y++)
    {
        for (size_t x = 0; x < num_anchor_x_inds; x++)
        {
            for (size_t r = 0; r < num_anchor_r_inds; r++)
            {
                int ind = y * num_anchor_x_inds * num_anchor_r_inds + x * num_anchor_r_inds + r;
                anchors_px_[ind] = anchor_x_count[x];
                anchors_py_[ind] = anchor_y_count[y];
                anchors_ro_[ind] = anchor_r_count[r];
                anchors_pz_[ind] = -1 * sensor_height;
                anchors_dx_[ind] = anchor_dx_size;
                anchors_dy_[ind] = anchor_dy_size;
                anchors_dz_[ind] = anchor_dz_size;
            }
        }
    }
}

void PointPillars::putAnchorsInDeviceMemory()
{
    // clang-format off
    GPU_CHECK(cudaMemcpy(dev_box_anchors_min_x_, box_anchors_min_x_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_box_anchors_min_y_, box_anchors_min_y_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_box_anchors_max_x_, box_anchors_max_x_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_box_anchors_max_y_, box_anchors_max_y_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    // clang-format on

    GPU_CHECK(cudaMemcpy(dev_anchors_px_, anchors_px_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_py_, anchors_py_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_pz_, anchors_pz_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_dx_, anchors_dx_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_dy_, anchors_dy_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_dz_, anchors_dz_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_anchors_ro_, anchors_ro_, num_anchor * sizeof(float), cudaMemcpyHostToDevice));
}

void PointPillars::convertAnchors2BoxAnchors(float* anchors_px, float* anchors_py, float* anchors_dx, float* anchors_dy,
                                             float* box_anchors_min_x_, float* box_anchors_min_y_,
                                             float* box_anchors_max_x_, float* box_anchors_max_y_)
{
    // flipping box's dimension
    float flipped_anchors_dx[num_anchor] = { 0 };
    float flipped_anchors_dy[num_anchor] = { 0 };
    for (size_t x = 0; x < num_anchor_x_inds; x++)
    {
        for (size_t y = 0; y < num_anchor_y_inds; y++)
        {
            int base_ind = x * num_anchor_y_inds * num_anchor_r_inds + y * num_anchor_r_inds;
            flipped_anchors_dx[base_ind + 0] = anchor_dx_size;
            flipped_anchors_dy[base_ind + 0] = anchor_dy_size;
            flipped_anchors_dx[base_ind + 1] = anchor_dy_size;
            flipped_anchors_dy[base_ind + 1] = anchor_dx_size;
        }
    }
    for (size_t x = 0; x < num_anchor_x_inds; x++)
    {
        for (size_t y = 0; y < num_anchor_y_inds; y++)
        {
            for (size_t r = 0; r < num_anchor_r_inds; r++)
            {
                int ind = x * num_anchor_y_inds * num_anchor_r_inds + y * num_anchor_r_inds + r;
                box_anchors_min_x_[ind] = anchors_px[ind] - flipped_anchors_dx[ind] / 2.0f;
                box_anchors_min_y_[ind] = anchors_py[ind] - flipped_anchors_dy[ind] / 2.0f;
                box_anchors_max_x_[ind] = anchors_px[ind] + flipped_anchors_dx[ind] / 2.0f;
                box_anchors_max_y_[ind] = anchors_py[ind] + flipped_anchors_dy[ind] / 2.0f;
            }
        }
    }
}

void PointPillars::initEngine_() {
    tensorrt_net_ptr_->DeserializeONNXEngine(pfe_onnx_file);
    tensorrt_net_ptr_->DeserializeONNXEngine(rpn_onnx_file);
}

void PointPillars::initEngine()
{
    // create a TensorRT model from the onnx model and serialize it to a stream
    nvinfer1::IHostMemory* pfe_trt_model_stream{ nullptr };
    nvinfer1::IHostMemory* rpn_trt_model_stream{ nullptr };
    onnxToTRTModel(pfe_onnx_file, pfe_trt_model_stream);
    onnxToTRTModel(rpn_onnx_file, rpn_trt_model_stream);
    if (pfe_trt_model_stream == nullptr || rpn_trt_model_stream == nullptr)
    {//use std:cerr instead of ROS_ERROR because want to keep this fille ros-agnostics
        std::cerr<< "Failed to load ONNX file " << std::endl;
    }

    // deserialize the engine
    pfe_runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    rpn_runtime_ = nvinfer1::createInferRuntime(trt_logger_);
    if (pfe_runtime_ == nullptr || rpn_runtime_ == nullptr)
    {
        std::cerr<<"Failed to create TensorRT Runtime object."<<std::endl;
    }
    pfe_engine_ =
            pfe_runtime_->deserializeCudaEngine(pfe_trt_model_stream->data(), pfe_trt_model_stream->size(), nullptr);
    rpn_engine_ =
            rpn_runtime_->deserializeCudaEngine(rpn_trt_model_stream->data(), rpn_trt_model_stream->size(), nullptr);
    if (pfe_engine_ == nullptr || rpn_engine_ == nullptr)
    {
        std::cerr << "Failed to create TensorRT Engine." << std::endl;
    }
    pfe_trt_model_stream->destroy();
    rpn_trt_model_stream->destroy();
    pfe_context_ = pfe_engine_->createExecutionContext();
    rpn_context_ = rpn_engine_->createExecutionContext();
    if (pfe_context_ == nullptr || rpn_context_ == nullptr)
    {
        std::cerr << "Failed to create TensorRT Execution Context." << std::endl;;
    }

}

//#ifndef TVM_IMPLEMENTATION
void PointPillars::onnxToTRTModel(const std::string& model_file,             // name of the onnx model
                                  nvinfer1::IHostMemory*& trt_model_stream)  // output buffer for the TensorRT model
{
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
    builder->setMaxBatchSize(batch_size);
    //builder->setMaxWorkspaceSize(1 << 20);
    config->setMaxWorkspaceSize(1 << 20);

    //nvinfer1::ICudaEngine* engine = builder->buildCudaEngine(*network);
    nvinfer1::ICudaEngine* engine = builder->buildEngineWithConfig(*network,*config);

    parser->destroy();

    // serialize the engine, then close everything down
    trt_model_stream = engine->serialize();
    engine->destroy();
    network->destroy();
    builder->destroy();
}
//#endif

void PointPillars::preprocessCPU(const float* in_points_array, const int in_num_points)
{
    int x_coors[max_num_pillars] = { 0 };
    int y_coors[max_num_pillars] = { 0 };
    float num_points_per_pillar[max_num_pillars] = { 0 };
    float* pillar_x = new float[max_num_pillars * max_num_points_per_pillar];
    float* pillar_y = new float[max_num_pillars * max_num_points_per_pillar];
    float* pillar_z = new float[max_num_pillars * max_num_points_per_pillar];
    float* pillar_i = new float[max_num_pillars * max_num_points_per_pillar];

    float* x_coors_for_sub_shaped = new float[max_num_pillars * max_num_points_per_pillar];
    float* y_coors_for_sub_shaped = new float[max_num_pillars * max_num_points_per_pillar];
    float* pillar_feature_mask = new float[max_num_pillars * max_num_points_per_pillar];

    float* sparse_pillar_map = new float[num_inds_for_scan * num_inds_for_scan];

    preprocess_points_ptr_->preprocess(in_points_array, in_num_points, x_coors, y_coors, num_points_per_pillar, pillar_x,
                                       pillar_y, pillar_z, pillar_i, x_coors_for_sub_shaped, y_coors_for_sub_shaped,
                                       pillar_feature_mask, sparse_pillar_map, host_pillar_count_);

    GPU_CHECK(cudaMemset(dev_x_coors_, 0, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_y_coors_, 0, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_pillar_x_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_y_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_z_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_i_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_x_coors_for_sub_shaped_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_y_coors_for_sub_shaped_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0, max_num_pillars * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0, num_inds_for_scan * num_inds_for_scan * sizeof(int)));

    // clang-format off
    GPU_CHECK(cudaMemcpy(dev_x_coors_, x_coors, max_num_pillars * sizeof(int), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_y_coors_, y_coors, max_num_pillars * sizeof(int), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_pillar_x_, pillar_x, max_num_pillars * max_num_points_per_pillar * sizeof(float),cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_pillar_y_, pillar_y, max_num_pillars * max_num_points_per_pillar * sizeof(float),cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_pillar_z_, pillar_z, max_num_pillars * max_num_points_per_pillar * sizeof(float),cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_pillar_i_, pillar_i, max_num_pillars * max_num_points_per_pillar * sizeof(float),cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_x_coors_for_sub_shaped_, x_coors_for_sub_shaped, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_y_coors_for_sub_shaped_, y_coors_for_sub_shaped,max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_num_points_per_pillar_, num_points_per_pillar, max_num_pillars * sizeof(float),cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_pillar_feature_mask_, pillar_feature_mask,max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyHostToDevice));
    GPU_CHECK(cudaMemcpy(dev_sparse_pillar_map_, sparse_pillar_map,num_inds_for_scan * num_inds_for_scan * sizeof(float), cudaMemcpyHostToDevice));
    // clang-format on

    delete[] pillar_x;
    delete[] pillar_y;
    delete[] pillar_z;
    delete[] pillar_i;
    delete[] x_coors_for_sub_shaped;
    delete[] y_coors_for_sub_shaped;
    delete[] pillar_feature_mask;
    delete[] sparse_pillar_map;
}

void PointPillars::preprocessGPU(const float* in_points_array, const int in_num_points)
{
    float* dev_points;
    // clang-format off
    GPU_CHECK(cudaMalloc((void**)&dev_points, in_num_points * num_box_corners * sizeof(float)));
    GPU_CHECK(cudaMemcpy(dev_points, in_points_array, in_num_points * num_box_corners * sizeof(float), cudaMemcpyHostToDevice));
    // clang-format on

    GPU_CHECK(cudaMemset(dev_pillar_count_histo_, 0, grid_y_size * grid_x_size * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_sparse_pillar_map_, 0, num_inds_for_scan * num_inds_for_scan * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_pillar_x_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_y_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_z_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_pillar_i_, 0, max_num_pillars * max_num_points_per_pillar * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_x_coors_, 0, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_y_coors_, 0, max_num_pillars * sizeof(int)));
    GPU_CHECK(cudaMemset(dev_num_points_per_pillar_, 0, max_num_pillars * sizeof(float)));
    GPU_CHECK(cudaMemset(dev_anchor_mask_, 0, num_anchor * sizeof(int)));

    preprocess_points_cuda_ptr_->doPreprocessPointsCuda(
            dev_points, in_num_points, dev_x_coors_, dev_y_coors_, dev_num_points_per_pillar_, dev_pillar_x_, dev_pillar_y_,
            dev_pillar_z_, dev_pillar_i_, dev_x_coors_for_sub_shaped_, dev_y_coors_for_sub_shaped_, dev_pillar_feature_mask_,
            dev_sparse_pillar_map_, host_pillar_count_);

    GPU_CHECK(cudaFree(dev_points));
}

void PointPillars::preprocess(const float* in_points_array, const int in_num_points)
{
    if (reproduce_result_mode)
    {
        preprocessCPU(in_points_array, in_num_points);
    }
    else
    {
        preprocessGPU(in_points_array, in_num_points);
    }
}

void PointPillars::Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,  std::vector<Box3D>& predResult)
{

    preprocess(in_points_array, in_num_points);

    anchor_mask_cuda_ptr_->doAnchorMaskCuda(dev_sparse_pillar_map_, dev_cumsum_along_x_, dev_cumsum_along_y_,
                                            dev_box_anchors_min_x_, dev_box_anchors_min_y_, dev_box_anchors_max_x_,
                                            dev_box_anchors_max_y_, dev_anchor_mask_);


    GPU_CHECK(cudaStreamCreate(&stream_));



    // clang-format off
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[0], dev_pillar_x_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[1], dev_pillar_y_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[2], dev_pillar_z_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[3], dev_pillar_i_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[4], dev_num_points_per_pillar_, max_num_pillars * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[5], dev_x_coors_for_sub_shaped_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[6], dev_y_coors_for_sub_shaped_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    GPU_CHECK(cudaMemcpyAsync(pfe_buffers_[7], dev_pillar_feature_mask_, max_num_pillars * max_num_points_per_pillar * sizeof(float), cudaMemcpyDeviceToDevice, stream_));
    // clang-format on

    pfe_context_->enqueue(batch_size, pfe_buffers_, stream_, nullptr);
    //rpn_context_->enqueueV2(rpn_buffers_, stream_, nullptr);

    GPU_CHECK(cudaMemset(dev_scattered_feature_, 0, rpn_input_size * sizeof(float)));

    scatter_cuda_ptr_->doScatterCuda(host_pillar_count_[0], dev_x_coors_, dev_y_coors_, (float*)pfe_buffers_[8],
                                     dev_scattered_feature_);


    GPU_CHECK(cudaMemcpyAsync(rpn_buffers_[0], dev_scattered_feature_, batch_size * rpn_input_size * sizeof(float),
                              cudaMemcpyDeviceToDevice, stream_));


    rpn_context_->enqueue(batch_size, rpn_buffers_, stream_, nullptr);
    //rpn_context_->enqueueV2(rpn_buffers_, stream_, nullptr);


    GPU_CHECK(cudaMemset(dev_filter_count_, 0, sizeof(int)));
    postprocess_cuda_ptr_->doPostprocessCuda(
            (float*)rpn_buffers_[1], (float*)rpn_buffers_[2], (float*)rpn_buffers_[3], dev_anchor_mask_, dev_anchors_px_,
            dev_anchors_py_, dev_anchors_pz_, dev_anchors_dx_, dev_anchors_dy_, dev_anchors_dz_, dev_anchors_ro_,
            dev_filtered_box_, dev_filtered_score_, dev_filtered_cls_, dev_filtered_dir_, dev_box_for_nms_, dev_filter_count_, num_classes,
            out_detections);


    // release the stream and the buffers
    cudaStreamDestroy(stream_);

}


