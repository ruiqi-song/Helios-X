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

#include "transform_process_node.h"


bool TransformProcessNode::InitMessage() {
    MessageManager::AddGpsOdomCallback(&TransformProcessNode::OdomCallback, this);
    //sub_gps = nh.subscribe<nav_msgs::Odometry>("/gps/odom",10, &TransformProcessNode::OdomCallback, this);
    return true;
}

bool TransformProcessNode::InitConfig(YAML::Node& config) {
    //Init();

}

bool TransformProcessNode::InitModule() {}

bool TransformProcessNode::InitWorkerNode(YAML::Node& config) {
    InitConfig(config);
    InitMessage();
    return true;
}

void TransformProcessNode::RegistAllAlgorithms() {
}

bool TransformProcessNode::InitSharedData() {
    gps_point_item_.reset(new GPSPoint());
}

void TransformProcessNode::CacheGPS(const nav_msgs::Odometry &gps, const double &timestamp) {
    gps_point_item_.reset(new GPSPoint());

    gps_point_item_->x = gps.pose.pose.position.x;
    gps_point_item_->y = gps.pose.pose.position.y;
    gps_point_item_->z = gps.pose.pose.position.z;
    gps_point_item_->timestamp = timestamp;

    gps_shared_data_->add(gps_point_item_);
}

void TransformProcessNode::OdomCallback(const nav_msgs::Odometry &gps) {

    map_vehicle_tf_.setData(tf::Transform(
            tf::Quaternion(gps.pose.pose.orientation.x,gps.pose.pose.orientation.y, gps.pose.pose.orientation.z,gps.pose.pose.orientation.w),
            tf::Vector3(gps.pose.pose.position.x,gps.pose.pose.position.y,gps.pose.pose.position.z)));

    map_vehicle_tf_.frame_id_ = "world_link";
    map_vehicle_tf_.child_frame_id_ = "base_link";
    map_vehicle_tf_.stamp_ = ros::Time::now();
    br.sendTransform(map_vehicle_tf_);

    CacheGPS(gps, gps.header.stamp.toSec());



    //Planning(gps);


    //gps
//    std::ofstream OutFile("/home/ricky/sunny.txt", std::ios::app); //利用构造函数创建txt文本，并且打开该文本
//    OutFile << gps.pose.pose.position.x << " " << gps.pose.pose.position.y << " " << gps.pose.pose.position.z << std::endl;



}


