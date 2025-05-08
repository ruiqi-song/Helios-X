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

CameraSharedObstacle *WorkerNode::camera_shared_obstacle_ = new CameraSharedObstacle();
LidarSharedObstacle *WorkerNode::lidar_shared_obstacle_ = new LidarSharedObstacle();
LidarSharedData *WorkerNode::lidar_shared_data_ = new LidarSharedData();
CameraSharedData *WorkerNode::camera_shared_data_ = new CameraSharedData();
CameraSharedData *WorkerNode::camera_shared_raw_ = new CameraSharedData();
TrajectorySharedData *WorkerNode::trajectory_shared_data_ = new TrajectorySharedData();
FusionSharedObstacle *WorkerNode::fusion_shared_obstacle_ = new FusionSharedObstacle();
GPSPointsSharedData *WorkerNode::gps_shared_data_ = new GPSPointsSharedData();


tf::Transform WorkerNode::camera_livox_ = read_extrinsics("camera_short_link");
tf::Transform WorkerNode::livox_vehicle_ = read_extrinsics("livox_lidar_link");
tf::Transform WorkerNode::os_vehicle_ = read_extrinsics("os_lidar_link");
tf::Transform WorkerNode::front_radar_vehicle_ = read_extrinsics("front_radar_link");
tf::Transform WorkerNode::left_radar_vehicle_ = read_extrinsics("left_radar_link");
tf::Transform WorkerNode::right_radar_vehicle_ = read_extrinsics("right_radar_link");
tf::Transform WorkerNode::imu_vehicle_ = read_extrinsics("imu_link");
tf::Transform WorkerNode::camera_vehicle_ = (livox_vehicle_ * camera_livox_.inverse()).inverse();

cv::Mat WorkerNode::camera_instrinsics_ = get_intrinsics();
cv::Mat WorkerNode::distortion_coefficients_ = get_distortion();

float WorkerNode::fx_ = camera_instrinsics_.at<double>(0);
float WorkerNode::fy_ = camera_instrinsics_.at<double>(4);
float WorkerNode::cx_ = camera_instrinsics_.at<double>(2);
float WorkerNode::cy_ = camera_instrinsics_.at<double>(5);

void WorkerNode::Run(){

    if (!inited_) {
        gLogWarning << "Subnode not inited, run failed. node: <" << id_ << ", " << name_
                    << ">" << std::endl;
        return;
    }

}

void WorkerNode::Init() {

}
