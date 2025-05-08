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


static std::string camera_image_raw1_topic = "/pylon_camera_node1/image_raw";
static std::string lidar_point_cloud_topic1 = "/os_cloud_node/points";

static std::string RECEIVE_ONLY = "receive_only";
static std::string PUBLISH_ONLY = "publish_only";
static std::string DUPLEX = "duplex";

//static std::string camera_image_raw_topic = "/pylon_camera_node1/image_raw"; //"/kitti/camera_color_left/image_raw"
static std::string camera_image_raw_topic = "/pylon_camera_node1/image_raw"; //"/kitti/camera_color_left/image_raw"
static std::string lidar_point_cloud_topic = "/livox/lidar";      //"/livox/lidar"/os_cloud_node/points  /velodyne_points /lidar_top
//static std::string lidar_point_cloud_topic = "/livox/lidar";
static std::string radar_point_cloud_topic = "/radar/points";
static std::string gps_odom_topic = "/gps/odom";
static std::string semantic_point_cloud_topic = "/semantic/points";
static std::string semantic_image_topic = "/semantic/image";

static std::string camera_obstacles_topic = "/camera/obstacles";
static std::string lidar_obstacles_topic = "/lidar/obstacles";
static std::string radar_obstacles_topic = "/radar/obstacles";
static std::string fusion_obstacles_topic = "/fusion/obstacles";
static std::string lane_obstacles_topic = "/lane/obstacles";
static std::string predict_trajectory_topic = "/predict/trajectory";


static std::string visual_camera_obstacles_topic = "/visual/camera/obstacles";
static std::string visual_lidar_obstacles_topic = "/visual/lidar/obstacles";
static std::string visual_fusion_obstacles_topic = "/visual/fusion/obstacles";
static std::string visual_history_points_topic = "/visual/history/points";
static std::string visual_history_trajectory_topic = "/visual/history/trajectory";
static std::string visual_predict_trajectory_topic = "/visual/predict/trajectory";
static std::string visual_segment_contour_topic = "/visual/semantic/contour/viewer";
static std::string visual_segment_project_topic = "/visual/semantic/project/viewer";
static std::string visual_fusion_image_topic = "/visual/fusion/image/viewer";
static std::string visual_camera_image_topic = "/visual/camera/image/viewer";
static std::string visual_self_topic = "/visual/self/obstacles";

