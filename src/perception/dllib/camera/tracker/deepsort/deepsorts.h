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

#include <iostream>>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <chrono>

#include <cv_bridge/cv_bridge.h>

#include "camera/tracker/deepsort/lib/deepsort.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include "camera/tracker/deepsort/lib/loggings.h"
//#include "inference/tensorrt/logging.h"
#include <ctime>

#include "yaml-cpp/yaml.h"

#include "base/camera_base_detector.h"
#include "base/camera_base_tracker.h"
#include "register/register.h"
#include "monitor/log.h"
#include "config/config.h"

class DeepSorts : public CameraBaseTracker{
public:
    DeepSorts();
    ~DeepSorts();
    void Init(YAML::Node& config) override ;
    void Track(cv_bridge::CvImagePtr &in_img_raw, std::vector<CameraDetector::Detection> &dets, std::vector<DetectBox> &outputs) override ;

private:

    //std::string engine_path = "/home/ricky/waytousLIB/track/DeepSORT/deepsort_ros/src/deepsort_tensorrt/resources/deepsort.engine";

    //vector<vector<DetectBox>> allDetections;

    //vector<DetectBox> out;

    DeepSort* DS;

   // waytous_msgs::DetectedObject singleObject;

//    void PostProcess(cv::Mat &img, vector<DetectBox> &mres, vector<DetectBox> &sres,
//                     waytous_msgs::DetectedObjectArray & out_objects);

    bool SetState(const int &class_id);

    std::unique_ptr<ConfigParser> config_;


};

REGISTER_CAMERA_TRACKER(DeepSorts);





