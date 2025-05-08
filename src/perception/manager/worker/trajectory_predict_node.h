//
// Created by ricky on 2021/12/9.
//

#pragma once

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "math/algebra.h"
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

#include "worker_node.h"

#include "message/rosmsgs/obstacles/DetectedObject.h"
#include "message/rosmsgs/obstacles/DetectedObjectArray.h"

#include "interface/message_manager.h"

struct GPSPoints {
    //sensor_msgs::Image data;
    float x;
    float y;
    float z;
};

class TrajectoryPredictNode : public WorkerNode{
public:
    TrajectoryPredictNode() = default;
    ~TrajectoryPredictNode() = default;


private:
    bool InitConfig(YAML::Node& config);
    bool InitMessage();
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);
    bool InitSharedData();
    void RegistAllAlgorithms() override;

    void TrajectoryCallback(const waytous_msgs::DetectedObjectArray::ConstPtr &fusion_obstacles);

    void PublishTrajectory();

    void GetHistoryTrajectory(ros::Time &stamp);
    bool ShowHistoryTrajectory(std::vector<waytous_msgs::DetectedObject> &vec, visualization_msgs::Marker &line,
                        visualization_msgs::Marker &points, visualization_msgs::Marker &array);


    void readTxt(std::string file);

    bool ShowTrajectory(std::vector<GPSPoints> &vec, visualization_msgs::Marker &line,
                        visualization_msgs::Marker &points, int &location, float &speed);
    bool ShowSelfWithSpeed(VisualMarkerAdapter::MessageType &text, VisualMarkerAdapter::MessageType &self, float &speed);

    bool GetTrajectory(std::vector<GPSPoints> &gps_points, std::vector<GPSPoints> & planning_points);

    bool Planning(GPSPoints & position);

    std::vector<GPSPoints> gps_points_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_fusion_obstacles_;
    ros::Publisher trajectory_publisher_;
    ros::Publisher points_publisher_;
    ros::Publisher predict_publisher_;






};

