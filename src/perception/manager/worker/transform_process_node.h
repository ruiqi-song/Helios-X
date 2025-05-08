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


#include<iostream>
#include<fstream>

#include<queue>
#include<vector>

#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

//#include <Eigen/Core>
//#include <Eigen/Geometry>

#include "yaml-cpp/yaml.h"

#include "config/config.h"
#include "worker_node.h"

#include "interface/message_manager.h"

#include "math/algebra.h"



class TransformProcessNode : public WorkerNode{
public:
    TransformProcessNode() = default;
    ~TransformProcessNode() = default;


private:
    bool InitConfig(YAML::Node& config);
    bool InitMessage();
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);
    bool InitSharedData();
    void RegistAllAlgorithms() override;

    void OdomCallback(const nav_msgs::Odometry &gps);

    void CacheGPS(const nav_msgs::Odometry &gps, const double &timestamp);


    tf::TransformBroadcaster br;

    tf::StampedTransform map_vehicle_tf_;



};
