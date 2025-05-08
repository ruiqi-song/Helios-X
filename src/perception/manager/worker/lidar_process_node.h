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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

#include<visualization_msgs/Marker.h>
#include<visualization_msgs/MarkerArray.h>

#include "interface/message_manager.h"

#include "monitor/log.h"

#include "worker_node.h"
#include "base/lidar_base_detector.h"
#include "base/lidar_base_tracker.h"
#include "container/ioc_container.h"
#include "lidar/detector/pointpillars/point_pillars.h"
#include "lidar/detector/centerpoint/centerpoint.h"
#include "lidar/detector/pointpillarsv2/pointpillars_multi_heads.h"
#include "lidar/detector/pointpillarsv2/pointpillars_nuscence.h"
#include "lidar/tracker/kalman/kalman.h"

#include "cache/camera_shared_obstacle.h"

class LidarProcessWorkerNode : public WorkerNode {
public:
    LidarProcessWorkerNode();
    ~LidarProcessWorkerNode() = default;

private:
    bool InitConfig(YAML::Node& config);
    bool InitMessage();
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);
    bool InitSharedData();
    void RegistAllAlgorithms();
    void CachePointCloud(const PointCloud2Adapter::MessageType &pc_ptr, const double &timestamp);
    void CacheLidarObstacle(const LidarObstaclesAdapter::MessageType &objects, const double &timestamp);


    void PointCloud2Callback(const PointCloud2Adapter::MessageType& msg);
    void LidarTrackeCallback();

    void pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array,
                    const float offset_z = 0);

    void getBaselinkToLidarTF(const std::string& target_frameid);

    void analyzeTFInfo(tf::StampedTransform lidar2baselink);

    geometry_msgs::Pose getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf);

    void pubDetectedObject(const std::vector<float>& detections, const std_msgs::Header& in_header);
    void pubDetectedObject_(const std::vector<Box3D>& detections, const std_msgs::Header& in_header);
    bool Visualization(std::vector<Box3D>& out_detection, const std_msgs::Header &in_header);


    //IocContainer ioc;
    std::unique_ptr<LidarBaseDetector> detector_;
    std::unique_ptr<LidarBaseTracker> tracker_;



    // initializer list
    ros::NodeHandle private_nh_;
    bool has_subscribed_baselink_;
    const int NUM_POINT_FEATURE_;
    const int OUTPUT_NUM_BOX_FEATURE_;
    const float TRAINED_SENSOR_HEIGHT_;
    const float NORMALIZING_INTENSITY_VALUE_;
    const std::string BASELINK_FRAME_;
    // end initializer list

    // rosparam
    bool baselink_support_;

    tf::TransformListener tf_listener_;
    tf::StampedTransform baselink2lidar_;
    tf::Transform angle_transform_;
    tf::Transform angle_transform_inversed_;

    float offset_z_from_trained_data_;

    std::unique_ptr<PointPillars> point_pillars_ptr_;

    YAML::Node lidar_config;


};