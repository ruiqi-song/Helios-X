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
#include <tf/tf.h>

#include "yaml-cpp/yaml.h"

#include "config/config.h"
#include "monitor/log.h"

#include "cache/common_shared_data.h"
#include "cache/camera_shared_data.h"
#include "cache/lidar_shared_data.h"
#include "cache/camera_shared_obstacle.h"
#include "cache/lidar_shared_obstacle.h"
#include "cache/trajectory_shared_data.h"
#include "cache/fusion_shared_obstacle.h"
#include "cache/gps_shared_data.h"

class WorkerNode{
public:
    WorkerNode() = default;


    virtual ~WorkerNode(){}
    void Init();
    virtual bool InitConfig(YAML::Node& config) = 0;
    virtual bool InitWorkerNode(YAML::Node& config) = 0;
    void Stop() { stop_ = true; }
    int id() const { return id_; }
    std::string name() const { return name_; }
    virtual void RegistAllAlgorithms() = 0;


    std::shared_ptr<CameraItem> camera_item_;
   // std::shared_ptr<CameraItem> camera_item_raw_;
    std::shared_ptr<CameraObstacle> camera_obstalce_item_;
    std::shared_ptr<LidarItem> lidar_item_;
    std::shared_ptr<LidarObstacle> lidar_obstalce_item_;
    std::shared_ptr<TrajectoryItem> trajectory_item_;
    std::shared_ptr<FusionObstacle> fusion_obstacle_item_;
    std::shared_ptr<GPSPoint> gps_point_item_;

//    static std::shared_ptr<CameraSharedObstacle> camera_shared_obstacle_;
//    static std::shared_ptr<LidarSharedData> lidar_shared_data_;
//    static std::shared_ptr<LidarSharedObstacle> lidar_shared_obstacle_;

    static CameraSharedData *camera_shared_data_;
    static CameraSharedData *camera_shared_raw_;
    static CameraSharedObstacle *camera_shared_obstacle_;
    static LidarSharedData *lidar_shared_data_;
    static LidarSharedObstacle *lidar_shared_obstacle_;
    static FusionSharedObstacle *fusion_shared_obstacle_;
    static TrajectorySharedData *trajectory_shared_data_;
    static GPSPointsSharedData *gps_shared_data_;


//    static tf::StampedTransform camera_vehicle_tf_;
//    static tf::StampedTransform vehicle_lidar_tf_;


    static tf::Transform os_vehicle_;
    static tf::Transform livox_vehicle_;

    static tf::Transform imu_vehicle_;

    static tf::Transform camera_livox_;

    static tf::Transform camera_vehicle_;

    static tf::Transform front_radar_vehicle_;
    static tf::Transform left_radar_vehicle_;
    static tf::Transform right_radar_vehicle_;

    static cv::Mat camera_instrinsics_;
    static cv::Mat distortion_coefficients_;

    static float fx_;
    static float fy_;
    static float cx_;
    static float cy_;




protected:
    void Run();
    int id_ = 0;
    std::string name_;


private:
    volatile bool stop_ = false;
    bool inited_ = false;
    int total_count_ = 0;
    int failed_count_ = 0;
};





