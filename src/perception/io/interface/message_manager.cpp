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

#include "message_manager.h"

MessageManager::MessageManager(){}

int MessageManager::front = 2022;
float MessageManager::rear = 2022.02;
std::string MessageManager::left = "2022.02.25";

callback  MessageManager::FrontCameraCall_ = std::bind(&MessageManager::FrontCameraCall, std::ref(MessageManager::front));
callback  MessageManager::RearCameraCall_ = std::bind(&MessageManager::RearCameraCall, std::ref(MessageManager::rear));
callback  MessageManager::LeftCameraCall_ = std::bind(&MessageManager::LeftCameraCall, std::ref(MessageManager::left));

MessageManager::new_map MessageManager::callback_map = BuildMap();

MessageManager::new_map MessageManager::BuildMap() {

    new_map callback_map_;

    callback_map_.insert(std::make_pair("FrontCamera", FrontCameraCall_));
    callback_map_.insert(std::make_pair("RearCamera", RearCameraCall_));
    callback_map_.insert(std::make_pair("LeftCamera", LeftCameraCall_));


    return callback_map_;
}


void MessageManager::FrontCameraCall(const int &w) {
    std::cout << "FrontCameraCall: " << w << std::endl;
}

void MessageManager::RearCameraCall(const float &w) {
    std::cout << "RearCameraCall" << w << std::endl;
}

void MessageManager::LeftCameraCall(const std::string &w) {
    std::cout << "LeftCameraCall" << w << std::endl;
}

void MessageManager::GetFuncByName(std::string &name) {
    auto &callback = callback_map[name];
    //std::cout << "callback_map size: " << callback_map.begin()->first << std::endl;
    callback();
//    for(auto& fun : vecs) {
//        fun();
//    }
}

void MessageManager::Init() {

//    Wrapper<int>::CallbackFun addCallBack = MessageManager::FrontCameraCall;
//    Wrapper<float>::CallbackFun addCallBack1 = MessageManager::RearCameraCall;
    std::cout << "init message start" << std::endl;


    instance()->node_handle_.reset(new ros::NodeHandle());
    InitImageRaw("ImageRaw", camera_image_raw_topic, "receive_only");
    InitFrontLongCamera("FrontLongCamera", camera_image_raw_topic, "receive_only");
    InitFrontShortCamera("FrontShortCamera", camera_image_raw1_topic, "receive_only");

    //InitPointCloud2("PointCloud2", lidar_point_cloud_topic1, "receive_only");
    InitHorizon("PointCloud2", lidar_point_cloud_topic, "receive_only");
    InitFrontRadar("RadarCan", radar_point_cloud_topic, "receive_only");
    InitGpsOdom("GpsOdom", gps_odom_topic, "receive_only");
    InitSemanticPointCloud2("SemanticPointCloud2", semantic_point_cloud_topic, "duplex");
    InitSemanticImageRaw("SemanticImageRaw", semantic_image_topic, "duplex");

    InitCameraObstacles("CameraObstacles", camera_obstacles_topic, "duplex");
    InitLidarObstacles("LidarObstacles", lidar_obstacles_topic, "duplex");
    InitRadarObstacles("RadarObstacles", radar_obstacles_topic, "duplex");
    InitFusionObstacles("FusionObstacles", fusion_obstacles_topic, "duplex");
    InitLaneObstacles("LaneObstacles", lane_obstacles_topic, "duplex");
    InitPredictTrajectory("PredictTrajectory", predict_trajectory_topic, "publish_only");

    InitVisualCameraObstacles("VisualCameraObstacles", visual_camera_obstacles_topic, "publish_only");
    InitVisualLidarObstacles("VisualLidarObstacles", visual_lidar_obstacles_topic, "publish_only");
    InitVisualFusionObstacles("VisualFusionObstacles", visual_fusion_obstacles_topic, "publish_only");
    InitVisualHistoryPoints("VisualHistoryPoints", visual_history_points_topic, "publish_only");
    InitVisualHistoryTrajectory("VisualHistoryTrajectory", visual_history_trajectory_topic, "publish_only");
    InitVisualPredictTrajectory("VisualPredictTrajectory", visual_predict_trajectory_topic, "publish_only");
    InitVisualSegmentProject("VisualSegmentProject", visual_segment_project_topic, "publish_only");
    InitVisualSegmentContour("VisualSegmentContour", visual_segment_contour_topic, "publish_only");
    InitVisualFusionImage("VisualFusionImage", visual_fusion_image_topic, "publish_only");
    InitVisualCameraImage("VisualCameraImage", visual_camera_image_topic, "publish_only");
    InitVisualSelf("VisualSelf", visual_self_topic, "publish_only");

    //std::cout << "init message end" << std::endl;
    //BuildMap();
   // std::cout << "build map end" << std::endl;

//    ::callbacks.insert(callbacks_t::value_type("FrontLongCamera",std::move()))

}
