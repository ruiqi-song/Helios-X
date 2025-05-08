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

#include "yaml-cpp/yaml.h"
#include "register/register.h"

struct Box3D{
    float x;
    float y;
    float z;
    float l;
    float h;
    float w;
    float velX;
    float velY;
    float theta;
    float score;
    int cls;
    bool isDrop; // for nms
};

class LidarBaseDetector{
public:
    virtual void Init(YAML::Node& config) = 0;
    virtual void Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,  std::vector<Box3D>& predResult) = 0;
    //CameraBaseDetector();
    //~CameraBaseDetector();

};

REGISTER_REGISTERER(LidarBaseDetector);
#define REGISTER_LIDAR_DETECTOR(name) REGISTER_CLASS(LidarBaseDetector, name)