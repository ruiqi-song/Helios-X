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
#include <chrono>

#include "yaml-cpp/yaml.h"

#include "register/register.h"
#include "camera/detector/yolov5/yolohead.h"

#define NMS_THRESH 0.4
#define CONF_THRESH 0.5
namespace CameraDetector
{
    //static constexpr int CHECK_COUNT = 3;
    static constexpr float IGNORE_THRESH = 0.1f;

    static constexpr int MAX_OUTPUT_BBOX_COUNT = 1000;
    static constexpr int CLASS_NUM = 9;
    static constexpr int INPUT_H = 640;
    static constexpr int INPUT_W = 640;
//    static constexpr int CLASS_NUM = 9;
//    static constexpr int INPUT_H = 608;
//    static constexpr int INPUT_W = 608;

    static constexpr int LOCATIONS = 4;
    struct alignas(float) Detection {
        //center_x center_y w h
        float bbox[LOCATIONS];
        float conf;  // bbox_conf * cls_conf
        float class_id;
        //float trackID;
    };
    static const int OUTPUT_SIZE = MAX_OUTPUT_BBOX_COUNT * sizeof(Detection) / sizeof(float) + 1;
    //static const int OUTPUT_SIZE = 7001;
}

class CameraBaseDetector{
public:
    //CameraBaseDetector() {}
    //virtual ~CameraBaseDetector() {}
    virtual void Init(YAML::Node& config) = 0;
    virtual void Detect(cv::Mat &input, std::vector<CameraDetector::Detection>& output, int& batchSize) = 0;
    virtual void nms(std::vector<CameraDetector::Detection>& res, float *output, float conf_thresh, float nms_thresh) = 0;
    //CameraBaseDetector();
    //~CameraBaseDetector();
//private:
    //DISALLOW_COPY_AND_ASSIGN(CameraBaseDetector);


    void image2Vec(float *data, const cv::Mat &img)
    {  auto t_start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < CameraDetector::INPUT_H * CameraDetector::INPUT_W; i++){
            *(data + i) = img.at<cv::Vec3b>(i)[2] / 255.0;
            *(data+ i + CameraDetector::INPUT_H * CameraDetector::INPUT_W) = img.at<cv::Vec3b>(i)[1] / 255.0;
            *(data + i + 2 * CameraDetector::INPUT_H * CameraDetector::INPUT_W) = img.at<cv::Vec3b>(i)[0] / 255.0;
        }
    }
};

REGISTER_REGISTERER(CameraBaseDetector);
#define REGISTER_CAMERA_DETECTOR(name) REGISTER_CLASS(CameraBaseDetector, name)