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

#include "tf/tf.h"

#include "register/register.h"
#include "message/rosmsgs/obstacles/DetectedObject.h"
#include "message/rosmsgs/obstacles/DetectedObjectArray.h"

class PostBaseFusion{
public:
    virtual void Init(tf::Transform &livox_vehicle, tf::Transform &camera_vehicle,
                      cv::Mat camera_instrinsics, cv::Mat distortion_coefficients) = 0;
    //virtual void Fusion() = 0;

    virtual void Fusion(
            waytous_msgs::DetectedObjectArray &camera_obstacles,
            waytous_msgs::DetectedObjectArray &lidar_obstacles,
            waytous_msgs::DetectedObjectArray &radar_obstacles,
            waytous_msgs::DetectedObjectArray &fusion_obstacles,
            cv::Mat &image_raw,
            sensor_msgs::PointCloud2 &point_cloud,
            cv::Mat &fusion_image) = 0;
    //CameraBaseDetector();
    //~CameraBaseDetector();

};

REGISTER_REGISTERER(PostBaseFusion);
#define REGISTER_FUSION(name) REGISTER_CLASS(PostBaseFusion, name)