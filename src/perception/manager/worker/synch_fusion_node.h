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
#include <iostream>
#include <thread>

#include <cv_bridge/cv_bridge.h>

#include "worker_node.h"
#include "container/ioc_container.h"
#include "monitor/log.h"
#include "base/post_base_fusion.h"
#include "fusion/post_fusion.h"

#include "interface/message_manager.h"

class FusionProcessWorkerNode : public WorkerNode {
public:
    FusionProcessWorkerNode() = default;
    ~FusionProcessWorkerNode() = default;


private:
    bool InitConfig(YAML::Node& config);
    bool InitMessage();
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);
    void RegistAllAlgorithms();

    void FusionCallback(const LidarObstaclesAdapter::MessageType &in_obstacles);

    bool GetCacheObstacles(const SystemTimeAdapter::MessageType & timestamp);
    void UpdateMarkerArray(const FusionObstaclesAdapter::MessageType &object_array);
    void UpdateMarkerArray_(const FusionObstaclesAdapter::MessageType &object_array);

    void CacheFusionObstacles(const FusionObstaclesAdapter::MessageType &fusion_obstacles, const double &timestamp);

    void CacheTrajectory(const VisualFusionObstaclesAdapter::MessageType &markerArray, const double &timestamp);

    void GetTrajectory(SystemTimeAdapter::MessageType &stamp);
    void ShowTrajectory(std::vector<VisualMarkerAdapter::MessageType> &vec, VisualMarkerAdapter::MessageType &line,
                        VisualMarkerAdapter::MessageType &points);


    IocContainer ioc;
    std::unique_ptr<PostBaseFusion> fusion_;

    int count = 0;
    bool start_cache_;

    //sensor_msgs::Image image_raw_;

    //sensor_msgs::PointCloud2 point_cloud_;
    PointCloud2Adapter::MessageType point_cloud_;

    CameraObstaclesAdapter::MessageType camera_obstacles_;
    LidarObstaclesAdapter::MessageType lidar_obstacles_;
    RadarObstaclesAdapter::MessageType radar_obstacles_;
    FusionObstaclesAdapter::MessageType fusion_obstacles_;

    VisualFusionObstaclesAdapter::MessageType marker_array_;
    VisualHistoryPointsAdapter::MessageType marker_points_;
    VisualHistoryTrajectoryAdapter::MessageType marker_line_;

    cv::Mat image_raw_;
    cv::Mat fusion_image_;

};



