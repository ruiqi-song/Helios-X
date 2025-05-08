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

#include <vector>

#include <opencv2/opencv.hpp>


#include "register/register.h"
#include "base/camera_base_detector.h"

#include "inference/inference.h"
#include "inference/inference_factory.h"
#include "inference/tensorrt/tensorrt_net.h"
#include "yolohead.h"

#include "config/config.h"


class Yolov5 : public CameraBaseDetector{
public:
    Yolov5();
    ~Yolov5();
    void Init(YAML::Node& config) override ;
    void Detect(cv::Mat &input, std::vector<CameraDetector::Detection>& output, int& batchSize) override ;

private:
    void PreProcess(const cv::Mat &in_image, std::vector<float> &data);


    float iou(float lbox[4], float rbox[4]);
    static bool cmp(const CameraDetector::Detection& a, const CameraDetector::Detection& b);
    //void nms(std::vector<Yolo::Detection>& res, float *output);
    void nms(std::vector<CameraDetector::Detection>& res, float* output, float conf_thresh, float nms_thresh);

    std::unique_ptr<ConfigParser> config_;

    std::unique_ptr<Inference> inference_;

    std::unique_ptr<TensorRTNet> tensorrt_net_ptr_;

    const bool isBuildEngine = false;
    std::string model_name = "yolov5-panet-spp";
    float ignore_thresh = 0.1;
    int max_output_bbox_count = 1000;
    int class_num = 9;
    int input_h;
    int input_w = 640;
    float nms_thresh = 0.4;
    float conf_thresh = 0.5;

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
};
//REG_CLASS(CameraObstacleDetector)

REGISTER_CAMERA_DETECTOR(Yolov5);