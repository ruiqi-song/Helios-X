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

#include "synch_fusion_node.h"

bool FusionProcessWorkerNode::InitConfig(YAML::Node& config){
    return true;
}

bool FusionProcessWorkerNode::InitMessage() {

    MessageManager::AddLidarObstaclesCallback(&FusionProcessWorkerNode::FusionCallback, this);

    return true;

}

bool FusionProcessWorkerNode::InitModule(){

    fusion_.reset(PostBaseFusionRegisterer::GetInstanceByName("PostFusion"));
    fusion_->Init(livox_vehicle_, camera_vehicle_, camera_instrinsics_, distortion_coefficients_);

    trajectory_shared_data_->setCacheSize(20);

    return true;

}

bool FusionProcessWorkerNode::InitWorkerNode(YAML::Node& config) {
    InitConfig(config);
    InitModule();
    InitMessage();
    return true;
}

void FusionProcessWorkerNode::RegistAllAlgorithms(){

    //ioc.RegisterType<CameraBaseDetector, Yolov5>("camera_obstalce_detector");
    //ioc.RegisterType<CameraBaseTracker, DeepSort>("camera_obstacle_tracker");
    RegisterFactoryPostFusion();


}


void FusionProcessWorkerNode::FusionCallback(const LidarObstaclesAdapter::MessageType &in_obstacles){
    auto t_start = std::chrono::high_resolution_clock::now();

    ros::Time time_ = in_obstacles.header.stamp;

    if(GetCacheObstacles(time_)){
        fusion_->Fusion(camera_obstacles_, lidar_obstacles_, radar_obstacles_, fusion_obstacles_, image_raw_, point_cloud_, fusion_image_);

//        for(int i = 0; i < fusion_obstacles_.objects.size(); ++i){
//            fusion_obstacles_.objects[i].pose.orientation.x = 0;
//            fusion_obstacles_.objects[i].pose.orientation.y = 0;
//            fusion_obstacles_.objects[i].pose.orientation.z = 0;
//            fusion_obstacles_.objects[i].pose.orientation.w = 0;
//            fusion_obstacles_.objects[i].valid = true;
//            fusion_obstacles_.objects[i].dimensions.x = 10;
//            fusion_obstacles_.objects[i].dimensions.y = 5;
//            fusion_obstacles_.objects[i].dimensions.z = 6;
//            if(fusion_obstacles_.objects[i].pose.position.x <= 0 ||
//                    fusion_obstacles_.objects[i].pose.position.y <= 0||
//                    std::isnan(fusion_obstacles_.objects[i].pose.position.x) ||
//                    std::isnan(fusion_obstacles_.objects[i].pose.position.y) ||
//                    std::isnan(fusion_obstacles_.objects[i].pose.position.z)
//                ){
//                fusion_obstacles_.objects[i].pose.position.x = 1;
//                fusion_obstacles_.objects[i].pose.position.y = 1;
//                fusion_obstacles_.objects[i].pose.position.z = 2;
//            }
//
//        }

        UpdateMarkerArray_(fusion_obstacles_);
        //pub_objects_.publish(fusion_obstacles_);
        MessageManager::PublishFusionObstacles(fusion_obstacles_);

        auto dis = fusion_shared_obstacle_->getMsgBeforeTime(fusion_obstacles_.header.stamp);
//        if(abs(abs(dis->data.objects.data()->pose.position.x)-abs(fusion_obstacles_.objects.data()->pose.position.x)) < 50)
        CacheFusionObstacles(fusion_obstacles_, fusion_obstacles_.header.stamp.toSec());

        sensor_msgs::ImagePtr fusion_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", fusion_image_).toImageMsg();
        //pub_image_.publish(fusion_image_msg);
        MessageManager::PublishVisualFusionImage(*fusion_image_msg);

    }

    auto t_end = std::chrono::high_resolution_clock::now();
    float total = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    gLogInfo <<  "fusion process node exec time: " << total << " ms." << std::endl;




}

void FusionProcessWorkerNode::CacheTrajectory(const VisualFusionObstaclesAdapter::MessageType &markerArrays,
                                              const double &timestamp) {
    trajectory_item_.reset(new TrajectoryItem());

    trajectory_item_->data = markerArrays;
    trajectory_item_->timestamp = timestamp;
    trajectory_shared_data_->add(trajectory_item_);

}

bool FusionProcessWorkerNode::GetCacheObstacles(const SystemTimeAdapter::MessageType &timestamp) {

    if(camera_shared_obstacle_->size() != 0 && lidar_shared_obstacle_->size() != 0 && lidar_shared_data_->size() != 0 && camera_shared_data_->size() != 0)
    {
        camera_obstacles_ = camera_shared_obstacle_->getMsgNearestTime(timestamp)->data;
        lidar_obstacles_ = lidar_shared_obstacle_->getMsgNearestTime(timestamp)->data;
        point_cloud_ = lidar_shared_data_->getMsgNearestTime(timestamp)->data;
        image_raw_ = camera_shared_data_->getMsgNearestTime(timestamp)->data;

        //std::cout << "image_raw_: " << std::to_string(image_raw_.header.stamp.toSec()) << std::endl;
        return true;
    } else
        { return false;}

    //radar_obstacles_ = lidar_shared_obstacle_->getMsgNearestTime(timestamp)->data;
}

void FusionProcessWorkerNode::UpdateMarkerArray(const FusionObstaclesAdapter::MessageType &object_array){
    marker_array_.markers.clear();
    //text_array_.markers.clear();
    //ROS_INFO("case 1112");
    for (size_t i = 0; i < object_array.objects.size(); ++i) {

        auto object = object_array.objects[i];

        //ROS_INFO("case 111");
        visualization_msgs::Marker marker;
        visualization_msgs::Marker text;
        //marker.header.frame_id = range_detections_->header.frame_id;

        marker.header.frame_id = "base_link";
        //ROS_INFO("case 1112");
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker_cube";
        marker.id = i + 1; // i id
        //marker.id = object.id; // i id
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = object.pose.position.x;
        marker.pose.position.y = object.pose.position.y;
        marker.pose.position.z = object.pose.position.z;
//        marker.pose.orientation.x = 0;
//        marker.pose.orientation.y = 0;
//        marker.pose.orientation.z = 0;
//        marker.pose.orientation.w = 1;

        text.header.frame_id = "base_link";
        text.header.stamp = ros::Time::now();
        text.ns = "basic_shapes";
        text.id = i * 2 + 1;

        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = object.pose.position.x;
        text.pose.position.y = object.pose.position.y;

        text.pose.orientation.x = 0;
        text.pose.orientation.y = 0;
        text.pose.orientation.z = 0;
        text.pose.orientation.w = 1;
        text.text = object.label + " id:" + std::to_string(object.id);

        text.scale.x = 3;
        text.scale.y = 3;

        text.scale.z = 3;
        text.color.r = 1;
        text.color.g = 1;
        text.color.b = 1;
        text.color.a = 1;



       if(object.label == "rock")
        {
            marker.scale.x = object.dimensions.x =  2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 2; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1; //object.color.r;
            marker.color.g = 0; //object.color.g;
            marker.color.b = 0; //object.color.b;
            marker.lifetime = ros::Duration(3);
            text.lifetime = ros::Duration(3);
        }
        else if(object.label == "person")
        {
            marker.scale.x = object.dimensions.x ; //object.dimensions.x;
            marker.scale.y = object.dimensions.y ; //object.dimensions.y;
            marker.scale.z = object.dimensions.z ; //object.dimensions.z;
            marker.color.a = 0.4; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0;
            marker.color.g = 0.698;
            marker.color.b = 0.93;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "lorry")
        {
            marker.scale.x = object.dimensions.x; //object.dimensions.x;
            marker.scale.y = object.dimensions.y; //object.dimensions.y;
            marker.scale.z = object.dimensions.z; //object.dimensions.z;
            marker.color.a = 0.4; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0;
            marker.color.g = 0.75;
            marker.color.b = 1;
            marker.lifetime = ros::Duration(0.2);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "truck")
        {

            marker.scale.x = object.dimensions.x; //object.dimensions.x;
            marker.scale.y = object.dimensions.y; //object.dimensions.y;
            marker.scale.z = object.dimensions.z; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "sign")
        {
            marker.scale.x = object.dimensions.x = 2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 3; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0; //object.color.r;
            marker.color.g = 1; //object.color.g;
            marker.color.b = 1; //object.color.b;
            marker.lifetime = ros::Duration(1);
            text.lifetime = ros::Duration(1);
        }
        else if(object.label == "car")
        {
            marker.scale.x = object.dimensions.x; //object.dimensions.x;
            marker.scale.y = object.dimensions.y; //object.dimensions.y;
            marker.scale.z = object.dimensions.z; //object.dimensions.z;
            marker.color.a = 0.4; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "auxiliary")
        {
            marker.scale.x = object.dimensions.x ; //object.dimensions.x;
            marker.scale.y = object.dimensions.y ; //object.dimensions.y;
            marker.scale.z = object.dimensions.z ; //object.dimensions.z;
            marker.color.a = 0.4; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 0.392;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "warning")
        {
            marker.scale.x = object.dimensions.x = 2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 3; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1;
            marker.color.g = 0; //object.color.g;
            marker.color.b = 0; //object.color.b;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "excavator")
        {
            marker.scale.x = object.dimensions.x ; //object.dimensions.x;
            marker.scale.y = object.dimensions.y ; //object.dimensions.y;
            marker.scale.z = object.dimensions.z ; //object.dimensions.z;
            marker.color.a = 0.4; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0.5;
            marker.color.g = 1;
            marker.color.b = 1;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }

        text.pose.position.z = object.pose.position.z + marker.scale.z;

        if((abs(marker.pose.position.x) + abs(marker.pose.position.y)) < 300){
            marker_array_.markers.push_back(marker);
            marker_array_.markers.push_back(text);
        }
    }
    visualization_msgs::Marker marker;

    //marker.header.frame_id = range_detections_->header.frame_id;

    marker.header.frame_id = "base_link";
    //ROS_INFO("case 1112");
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_cube";
    marker.id = 0; // i id
    //marker.id = object.id; // i id
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 12; //object.dimensions.x;
    marker.scale.y = 6; //object.dimensions.y;
    marker.scale.z = 6; //object.dimensions.z;
    marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
    marker.color.r = 0.78; //object.color.r;
    marker.color.g = 0.08; //object.color.g;
    marker.color.b = 0.52; //object.color.b;
    marker.lifetime = ros::Duration(0.2);
    marker_array_.markers.push_back(marker);

    MessageManager::PublishVisualFusionObstacles(marker_array_);


    //marker_publisher_.publish(marker_array_);
}

void FusionProcessWorkerNode::CacheFusionObstacles(const FusionObstaclesAdapter::MessageType &fusion_obstacles,
                                                   const double &timestamp) {
    fusion_obstacle_item_.reset(new FusionObstacle());

    fusion_obstacle_item_->data = fusion_obstacles;
    fusion_obstacle_item_->timestamp = timestamp;
    fusion_shared_obstacle_->add(fusion_obstacle_item_);
}


void FusionProcessWorkerNode::GetTrajectory(SystemTimeAdapter::MessageType &stamp) {

    std::vector<std::vector<visualization_msgs::Marker>> trace_;

    std::vector<int> vec_id;

    marker_line_.markers.clear();
    marker_points_.markers.clear();

    //visualization_msgs::MarkerArray line_array_;
   // visualization_msgs::MarkerArray points_array_;

    auto this_objects = trajectory_shared_data_->getMsgNearestTime(stamp);


    for(int i = 0; i < this_objects->data.markers.size(); ++i){
        if(this_objects->data.markers[i].id != 0){
            vec_id.push_back(this_objects->data.markers[i].id);
        }
    }



    for(int k = 0; k < vec_id.size(); ++k){
        std::vector<visualization_msgs::Marker> vec_;
        for(int i = 0; i < trajectory_shared_data_->size(); i++){
            visualization_msgs::MarkerArray markerArray_ = trajectory_shared_data_->getMsgWithId(i)->data;
            for(int j = 0; j < markerArray_.markers.size(); j++){
                if(vec_id[k] == markerArray_.markers[j].id){
                    vec_.push_back(markerArray_.markers[j]);
                }

            }
        }
        trace_.push_back(vec_);
    }





    for(int i = 0; i < trace_.size(); ++i){

        visualization_msgs::Marker line_;
        visualization_msgs::Marker points_;
        if(trace_[i].size() > 1){
            ShowTrajectory(trace_[i], line_, points_);
        }

        marker_line_.markers.push_back(line_);
        marker_points_.markers.push_back(points_);
    }



    MessageManager::PublishVisualHistoryPoints(marker_points_);
    MessageManager::PublishVisualHistoryTrajectory(marker_line_);




    //trajectory_publisher_.publish(line_array_);
    //points_publisher_.publish(points_array_);



}

void FusionProcessWorkerNode::ShowTrajectory(std::vector<VisualMarkerAdapter::MessageType> &vec, VisualMarkerAdapter::MessageType &line,
                                             VisualMarkerAdapter::MessageType &points){

    for(int i = 1; i < vec.size() - 1; i++){
       // visualization_msgs::Marker line_, points_;
        line.header.frame_id = "base_link";
        points.header.frame_id = "base_link";
        line.header.stamp = ros::Time::now();
        points.header.stamp = ros::Time::now();
        line.ns = points.ns = "points_and_lins";
        line.action = points.action = visualization_msgs::Marker::ADD;


        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 0.5;
        line.color.g = 1;
        line.color.a = 1;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.r = 1.0f;
        points.color.a = 1.0;

        geometry_msgs::Point p, q;
        p.x = vec[i].pose.position.x;
        p.y = vec[i].pose.position.y;
        p.z = vec[i].pose.position.z;

        q.x = vec[i + 1].pose.position.x;
        q.y = vec[i + 1].pose.position.y;
        q.z = vec[i + 1].pose.position.z;

        line.points.push_back(p);
        line.points.push_back(q);
        points.points.push_back(p);
        points.points.push_back(q);

        line.lifetime = ros::Duration(0.2);
        points.lifetime = ros::Duration(0.2);


    }

}

void FusionProcessWorkerNode::UpdateMarkerArray_(const FusionObstaclesAdapter::MessageType &object_array){
    marker_array_.markers.clear();
    for (size_t i = 0; i < object_array.objects.size(); ++i) {

        auto object = object_array.objects[i];

        //ROS_INFO("case 111");
        visualization_msgs::Marker marker;
        visualization_msgs::Marker text;
        //marker.header.frame_id = range_detections_->header.frame_id;

        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "marker_cube";
        marker.id = i + 1; // i id
        //marker.id = object.id; // i id
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = object.pose.position.x;
        marker.pose.position.y = object.pose.position.y;
        marker.pose.position.z = object.pose.position.z + 4;
//        marker.pose.orientation.x = 0;
//        marker.pose.orientation.y = 0;
//        marker.pose.orientation.z = 0;
//        marker.pose.orientation.w = 1;

        text.header.frame_id = "base_link";
        text.header.stamp = ros::Time::now();
        text.ns = "basic_shapes";
        text.id = i * 2 + 1;

        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;


        text.pose.orientation.x = 0;
        text.pose.orientation.y = 0;
        text.pose.orientation.z = 0;
        text.pose.orientation.w = 1;
        text.text = object.label;
       // text.text = object.label + " id:" + std::to_string(object.id);

        text.scale.x = 6;
        text.scale.y = 6;

        text.scale.z = 6;
        text.color.r = 1;
        text.color.g = 0;
        text.color.b = 0;
        text.color.a = 1;

        if(object.label != "rock" && object.label != "sign" && object.label != "warning"){
            continue;
        }



        if(object.label == "rock")
        {
            marker.scale.x = object.dimensions.x = 2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 2; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1; //object.color.r;
            marker.color.g = 0; //object.color.g;
            marker.color.b = 0; //object.color.b;
            text.pose.position.x = object.pose.position.x;
            text.pose.position.y = object.pose.position.y;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }

        else if(object.label == "sign")
        {
            marker.scale.x = object.dimensions.x = 2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 8; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 0; //object.color.r;
            marker.color.g = 1; //object.color.g;
            marker.color.b = 1; //object.color.b;
            text.pose.position.x = object.pose.position.x;
            text.pose.position.y = object.pose.position.y;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        else if(object.label == "warning")
        {
            marker.scale.x = object.dimensions.x = 2; //object.dimensions.x;
            marker.scale.y = object.dimensions.y = 2; //object.dimensions.y;
            marker.scale.z = object.dimensions.z = 8; //object.dimensions.z;
            marker.color.a = 1; //object.color.a; // Don't forget to set the alpha!
            marker.color.r = 1;
            marker.color.g = 0.5; //object.color.g;
            marker.color.b = 0.5; //object.color.b;
            text.pose.position.x = object.pose.position.x;
            text.pose.position.y = object.pose.position.y;
            marker.lifetime = ros::Duration(0.5);
            text.lifetime = ros::Duration(0.5);
        }
        text.pose.position.z = object.pose.position.z + marker.scale.z + 1;

        if((abs(marker.pose.position.x) + abs(marker.pose.position.y)) < 300 && marker.pose.position.x != 0){
            marker_array_.markers.push_back(marker);
            marker_array_.markers.push_back(text);
        }

    }


    //marker.header.frame_id = range_detections_->header.frame_id;



    MessageManager::PublishVisualFusionObstacles(marker_array_);





}