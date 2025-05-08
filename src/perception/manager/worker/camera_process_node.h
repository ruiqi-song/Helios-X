/******************************************************************************
 * Copyright 2022 The Helios-X  Authors. All Rights Reserved.
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

#include <iostream>
#include <thread>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cudnn.h>
#include <pthread.h>

#include "register/register.h"

#include "worker_node.h"

#include "base/camera_base_detector.h"
#include "base/camera_base_tracker.h"

#include "camera/detector/yolov5/yolov5.h"
#include "camera/tracker/deepsort/deepsorts.h"

#include "camera/detector/yolov5/yolohead.h"
#include "inference/tensorrt/tensorrt_net.h"

#include "monitor/log.h"

#include "interface/message_manager.h"
#include "cache/camera_shared_obstacle.h"


class CameraProcessWorkerNode : public WorkerNode {
public:
    CameraProcessWorkerNode() = default;
    ~CameraProcessWorkerNode() = default;


private:
    bool InitConfig(YAML::Node& config);
    bool InitMessage(std::string& name);
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);
    bool InitSharedData();
    void RegistAllAlgorithms() override;

    void ImgCallback(const ImageRawAdapter::MessageType & in_image_msg);

    bool MessageToMat(const ImageRawAdapter::MessageType &msg, cv_bridge::CvImagePtr &cv_bridge_img, cv::Mat *img);
    bool MatToMessage(const cv::Mat& img, ImageRawAdapter::MessageType *msg);

    void CacheCameraObstacle(const CameraObstaclesAdapter ::MessageType &objects, const double &timestamp);
    void CacheImageRaw(const cv::Mat &image, const double &timestamp);
    void CacheImageViewer(const cv::Mat &image, const double &timestamp);

    bool ResizeMatImg(cv::Mat & img,  cv::Mat& resized_img);

    cv::Rect get_rect(cv::Mat& img, float bbox[4]);
    void RestoreBox(cv::Mat &img, std::vector<CameraDetector::Detection> &res);
    void PostProcess(cv_bridge::CvImagePtr &cv_img, std::vector<DetectBox> &res,
                     CameraObstaclesAdapter ::MessageType & out_objects);
    //void PostProcessTrack(cv::Mat &img, vector<DetectBox> &mres, vector<DetectBox> &sres,
    //                      waytous_msgs::DetectedObjectArray & out_objects);

    //IocContainer ioc;
    std::unique_ptr<CameraBaseDetector> detector_;
    std::unique_ptr<CameraBaseTracker> tracker_;

    const bool isBuildEngine = false;

    int batch_size = 1;
//    cv_bridge::CvImagePtr cv_bridge_img;

    CameraObstaclesAdapter::MessageType out_objects;
    PerceptionObstacleAdapter::MessageType singleObject;

    YAML::Node camera_config;

    std::string detector_id;
//    std::string detector_model;
//    std::string detector_conf;
    std::string tracker_id;
//    std::string tracker_model;
//    std::string tracker_conf;

    std::string callback_;

};

