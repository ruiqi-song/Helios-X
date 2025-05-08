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

#include <opencv2/opencv.hpp>

#include "yaml-cpp/yaml.h"

#include "register/register.h"
#include <vector>

namespace LaneSegment{
    static constexpr int INPUT_H = 640;
    static constexpr int INPUT_W = 640;
    static int iH = 512;
    static int iW = 1024;
    static int oH = 512;
    static int oW = 1024;
}

class LaneBaseSegment{
public:
    virtual void Init(YAML::Node& config) = 0;
    virtual void Segment(cv::Mat &in_image, std::vector<int> &res) = 0;

    virtual void PreProcess(cv::Mat &in_image,  std::vector<float> &data) = 0;

    void image2Vec(float * data, const cv::Mat & img)
    {
        for(int i = 0; i < LaneSegment::INPUT_H * LaneSegment::INPUT_W; i++){
            *(data + i) = img.at<cv::Vec3b>(i)[2] / 255.0;
            *(data+ i + LaneSegment::INPUT_H * LaneSegment::INPUT_W) = img.at<cv::Vec3b>(i)[1] / 255.0;
            *(data + i + 2 * LaneSegment::INPUT_H * LaneSegment::INPUT_W) = img.at<cv::Vec3b>(i)[0] / 255.0;
        }
    }



};

REGISTER_REGISTERER(LaneBaseSegment);
#define REGISTER_CAMERA_SEGMENTATION(name) REGISTER_CLASS(LaneBaseSegment, name)