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

#include "lidar_process_node.h"

LidarProcessWorkerNode::LidarProcessWorkerNode():
  private_nh_("~")
  , has_subscribed_baselink_(false)
  , NUM_POINT_FEATURE_(4)
  , OUTPUT_NUM_BOX_FEATURE_(7)
  , TRAINED_SENSOR_HEIGHT_(1.73f)
  , NORMALIZING_INTENSITY_VALUE_(255.0f)
  //, BASELINK_FRAME_("vehicle_link"){}
  //, BASELINK_FRAME_("vehicle_link"){}
  , BASELINK_FRAME_("vehicle_link"){}

bool LidarProcessWorkerNode::InitConfig(YAML::Node& config){
    lidar_config = config;
    return true;

}

bool LidarProcessWorkerNode::InitMessage() {

    MessageManager::AddHorizonCallback(&LidarProcessWorkerNode::PointCloud2Callback, this);
    return true;
}

bool LidarProcessWorkerNode::InitModule(){
    //detector_.reset(LidarBaseDetectorRegisterer::GetInstanceByName("PointPillars"));
    detector_.reset(LidarBaseDetectorRegisterer::GetInstanceByName("PointPillarsMultiHeads"));
    //detector_.reset(LidarBaseDetectorRegisterer::GetInstanceByName("PointPillarsNuScene"));
    //detector_.reset(LidarBaseDetectorRegisterer::GetInstanceByName("CenterPoint"));
    detector_->Init(lidar_config);
    tracker_.reset(LidarBaseTrackerRegisterer::GetInstanceByName("Kalman"));
    tracker_->Init(lidar_config);

    return true;

}

bool LidarProcessWorkerNode::InitSharedData() {

    lidar_obstalce_item_.reset(new LidarObstacle());


}

bool LidarProcessWorkerNode::InitWorkerNode(YAML::Node& config) {
    InitConfig(config);
    InitModule();
    InitMessage();

    return true;
}

void LidarProcessWorkerNode::CachePointCloud(const PointCloud2Adapter::MessageType &pc_ptr,
                                             const double &timestamp) {
    lidar_item_.reset(new LidarItem());

    lidar_item_->data = pc_ptr;
    lidar_item_->timestamp = timestamp;
    lidar_shared_data_->add(lidar_item_);
}


void LidarProcessWorkerNode::CacheLidarObstacle(const LidarObstaclesAdapter::MessageType &objects,
                                                const double &timestamp) {
    lidar_obstalce_item_.reset(new LidarObstacle());

    lidar_obstalce_item_->data = objects;
    lidar_obstalce_item_->timestamp = timestamp;
    lidar_shared_obstacle_->add(lidar_obstalce_item_);
}



void LidarProcessWorkerNode::PointCloud2Callback(const PointCloud2Adapter::MessageType &msg){

    auto t_start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(msg, *pcl_pc_ptr);


    CachePointCloud(msg, msg.header.stamp.toSec());
   // pcl_ros::transformPointCloud(*pcl_pc_ptr, *pcl_pc_ptr, livox_vehicle_.inverse());

    pcl_ros::transformPointCloud(*pcl_pc_ptr, *pcl_pc_ptr, livox_vehicle_);


    float* points_array = new float[pcl_pc_ptr->size() * NUM_POINT_FEATURE_];
    pclToArray(pcl_pc_ptr, points_array);

    std::vector<float> out_detection;
      std::vector<Box3D> predResult_;

    detector_->Detect(points_array, pcl_pc_ptr->size(), out_detection, predResult_);

    delete[] points_array;


     pubDetectedObject_(predResult_, msg.header);
    Visualization(predResult_, msg.header);
    auto t_end = std::chrono::high_resolution_clock::now();
    float total = std::chrono::duration<float, std::milli>(t_end- t_start).count();
    gLogInfo <<  "lidar process node exec time: " << total << " ms." << std::endl;

//    sensor_msgs::PointCloud2 cloud_msg;
//    pcl::toROSMsg(*pcl_pc_ptr, cloud_msg);
//
//    MessageManager::PublishSemanticPointCloud2(cloud_msg);



    //detector_->Detect(points_array, pcl_pc_ptr->size(), out_detection);
    //tracker_->Track();
}

void LidarProcessWorkerNode::LidarTrackeCallback(){
    //detector_->Detect();
    tracker_->Track();
}

void LidarProcessWorkerNode::RegistAllAlgorithms(){

    //ioc.RegisterType<CameraBaseDetector, Yolov5>("camera_obstalce_detector");
    //ioc.RegisterType<CameraBaseTracker, DeepSort>("camera_obstacle_tracker");
    //RegisterFactoryCenterPoint();
    //RegisterFactoryPointPillars();
    RegisterFactoryPointPillarsMultiHeads();
    //RegisterFactoryPointPillarsNuScene();
    RegisterFactoryKalman();

}



void LidarProcessWorkerNode::pubDetectedObject(const std::vector<float>& detections, const std_msgs::Header& in_header)
{
    //visualization_msgs::MarkerArray marker_array;
    LidarObstaclesAdapter::MessageType objects;
   // waytous_msgs::DetectedObjectArray lidar_objects;
    objects.header = in_header;
//    objects.header.frame_id = "os_lidar";
    objects.header.frame_id = in_header.frame_id;
    objects.header.stamp = in_header.stamp;

    int num_objects = detections.size() / OUTPUT_NUM_BOX_FEATURE_;
    for (size_t i = 0; i < num_objects; i++)
    {
        PerceptionObstacleAdapter::MessageType object;
        object.header = in_header;
        object.valid = true;
        object.pose_reliable = true;

        object.pose.position.x = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 0];
        object.pose.position.y = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 1];
        object.pose.position.z = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 2];

        // Trained this way
        float yaw = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 6];
        yaw += M_PI/2;
        yaw = std::atan2(std::sin(yaw), std::cos(yaw));
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(-yaw);
        object.pose.orientation = q;

        if (baselink_support_)
        {
            object.pose = getTransformedPose(object.pose, angle_transform_inversed_);
        }

        // Again: Trained this way
        object.dimensions.x = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 4];
        object.dimensions.y = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 3];
        object.dimensions.z = detections[i * OUTPUT_NUM_BOX_FEATURE_ + 5];

        //Only detects car in Version 1.0
        object.label = "car";

        objects.objects.push_back(object);

       // std::cout << object << std::endl;

        //publish marker
        //visualization_msgs::Marker marker;
        /*
        marker.header.frame_id = "IMU_link";
        //marker.header.stamp = ros::Time();
        marker.header.stamp = object.header.stamp;
        //marker.ns = "my_namespace";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = object.pose.position.x;
        marker.pose.position.y = object.pose.position.y;
        marker.pose.position.z = object.pose.position.z;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = object.dimensions.x;
        marker.scale.y = object.dimensions.y;
        marker.scale.z = object.dimensions.z;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        pub_marker_.publish(marker);

        //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
        marker_array.markers.push_back(marker);*/

        //publish end
    }

    CacheLidarObstacle(objects, in_header.stamp.toSec());

    //pub_objects_.publish(objects);
    MessageManager::PublishLidarObstacles(objects);

    //pub_lidar_objects_.publish(lidar_objects);
    //pub_marker_array_.publish(marker_array);
}

void LidarProcessWorkerNode::pubDetectedObject_(const std::vector<Box3D> &detections, const std_msgs::Header &in_header) {
    //visualization_msgs::MarkerArray marker_array;
    LidarObstaclesAdapter::MessageType objects;
    // waytous_msgs::DetectedObjectArray lidar_objects;
    objects.header = in_header;
//    objects.header.frame_id = "os_lidar";
    objects.header.frame_id = in_header.frame_id;
    objects.header.stamp = in_header.stamp;

    int num_objects = detections.size();
    for (size_t i = 0; i < num_objects; i++)
    {
        gLogInfo << "lidar object score: " << detections[i].score << std::endl;
        PerceptionObstacleAdapter::MessageType object;
        object.header = in_header;
        object.valid = true;
        object.pose_reliable = true;

        object.pose.position.x = detections[i].x;
        object.pose.position.y = detections[i].y;
        object.pose.position.z = detections[i].z;

        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(detections[i].theta);
       // gLogInfo << "<output yaw > :" << detections[i].theta << std::endl;
        object.pose.orientation.x = q.x;
        object.pose.orientation.y = q.y;
        object.pose.orientation.z = q.z;
        object.pose.orientation.w = q.w;


        // Again: Trained this way
        object.dimensions.x = detections[i].l;
        object.dimensions.y = detections[i].h;
        object.dimensions.z = detections[i].w;

        //Only detects car in Version 1.0
        object.label = detections[i].cls;

        objects.objects.push_back(object);


    }

    CacheLidarObstacle(objects, in_header.stamp.toSec());

    MessageManager::PublishLidarObstacles(objects);

}

geometry_msgs::Pose LidarProcessWorkerNode::getTransformedPose(const geometry_msgs::Pose& in_pose, const tf::Transform& tf)
{
    tf::Transform transform;
    geometry_msgs::PoseStamped out_pose;
    transform.setOrigin(tf::Vector3(in_pose.position.x, in_pose.position.y, in_pose.position.z));
    transform.setRotation(
            tf::Quaternion(in_pose.orientation.x, in_pose.orientation.y, in_pose.orientation.z, in_pose.orientation.w));
    geometry_msgs::PoseStamped pose_out;
    tf::poseTFToMsg(tf * transform, out_pose.pose);
    return out_pose.pose;
}

void LidarProcessWorkerNode::getBaselinkToLidarTF(const std::string& target_frameid)
{
    try
    {
        tf_listener_.waitForTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(BASELINK_FRAME_, target_frameid, ros::Time(0), baselink2lidar_);
        analyzeTFInfo(baselink2lidar_);
        has_subscribed_baselink_ = true;
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void LidarProcessWorkerNode::analyzeTFInfo(tf::StampedTransform baselink2lidar)
{
    tf::Vector3 v = baselink2lidar.getOrigin();
    offset_z_from_trained_data_ = v.getZ() - TRAINED_SENSOR_HEIGHT_;

    tf::Quaternion q = baselink2lidar_.getRotation();
    angle_transform_ = tf::Transform(q);
    angle_transform_inversed_ = angle_transform_.inverse();
}

void LidarProcessWorkerNode::pclToArray(const pcl::PointCloud<pcl::PointXYZI>::Ptr& in_pcl_pc_ptr, float* out_points_array,
                                 const float offset_z)
{
    for (size_t i = 0; i < in_pcl_pc_ptr->size(); i++)
    {
        pcl::PointXYZI point = in_pcl_pc_ptr->at(i);
        out_points_array[i * NUM_POINT_FEATURE_ + 0] = point.x;
        out_points_array[i * NUM_POINT_FEATURE_ + 1] = point.y;
        out_points_array[i * NUM_POINT_FEATURE_ + 2] = point.z + offset_z;

        //out_points_array[i * NUM_POINT_FEATURE_ + 3] = float(point.intensity / NORMALIZING_INTENSITY_VALUE_);
        //centerpoint
        out_points_array[i * NUM_POINT_FEATURE_ + 3] = float(point.intensity);
        //out_points_array[i * NUM_POINT_FEATURE_ + 4] = 0;


    }
}

bool LidarProcessWorkerNode::Visualization(std::vector<Box3D>& out_detection, const std_msgs::Header &in_header) {

    VisualLidarObstaclesAdapter::MessageType marker_array;
    VisualLidarObstaclesAdapter::MessageType self_array;

    //visualization_msgs::MarkerArray marker_array;
    //visualization_msgs::MarkerArray text_dis_marker_array;
    for(size_t j = 0; j < out_detection.size(); j++) {    //each object

        visualization_msgs::Marker marker;
        //visualization_msgs::Marker text_dis_marker;
        marker.header.frame_id = "vehicle_link";
        //marker.header.stamp = ros::Time::now();
        marker.header.stamp =  ros::Time::now();
        marker.ns = "box";
        marker.id = j+1;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = out_detection[j].x;// + 10.55;
        marker.pose.position.y = out_detection[j].y;//0.555;
        marker.pose.position.z = out_detection[j].z + 6;//0.15;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw((out_detection[j].theta));
        //gLogInfo << "<output yaw > :" << out_detection[j].theta << std::endl;

        marker.pose.orientation.x = q.x;
        marker.pose.orientation.y = q.y;
        marker.pose.orientation.z = q.z;
        marker.pose.orientation.w = q.w;
        // marker.pose.orientation.x = 0;
        // marker.pose.orientation.y = 0;
        // marker.pose.orientation.z = 0;
        // marker.pose.orientation.w = 1;
        marker.scale.x = out_detection[j].l;
        marker.scale.y = out_detection[j].w;
        marker.scale.z = out_detection[j].h;
//        marker.scale.x = 1;
//        marker.scale.y = 1;
//        marker.scale.z = 1;
        marker.color.a = 1; // Don't forget to set the alpha!
        if (0 == (int)out_detection[j].cls) {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        } else if (1 == (int)out_detection[j].cls) {
            marker.color.r = 0.0;
            marker.color.g = 0.392;
            marker.color.b = 0.0;
// marker.color.a = 0;
        } else if (2 == (int)out_detection[j].cls) {
            marker.color.r = 0.5;
            marker.color.g = 1;
            marker.color.b = 1;
        }else if (3 == (int)out_detection[j].cls) {
            marker.color.r = 0;
            marker.color.g = 0.75;
            marker.color.b = 1;
        }
        else if (4 == (int)out_detection[j].cls) {
            marker.color.r = 0;
            marker.color.g = 0;
            marker.color.b = 0.8;
        }
        else {
            marker.color.r = 0;
            marker.color.g = 0.698;
            marker.color.b = 0.93;
        }
        marker.lifetime = ros::Duration(0.5);

        marker_array.markers.push_back(marker);



//        text_dis_marker.color.a = 1.0; // Don't forget to set the alpha!
//        text_dis_marker.header.frame_id = "velo_link";
//        text_dis_marker.header.stamp = ros::Time::now();
//        text_dis_marker.ns = "text";
//        text_dis_marker.action = visualization_msgs::Marker::ADD;
//        text_dis_marker.pose.orientation.w = 1.0;
//        text_dis_marker.id = j;
//        text_dis_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//
//        text_dis_marker.scale.z = 1.5;
//        text_dis_marker.lifetime = ros::Duration(0.6);
//        std::ostringstream buffer;
//        buffer << out_detection[j].score ;
//        std::string conf = buffer.str();
//
//        geometry_msgs::Pose pose;
//        pose.position.x =
//                out_detection[j].x + out_detection[j].l / 2;
//        pose.position.y =
//                out_detection[j].y + out_detection[j].h / 2;
//        pose.position.z =
//                out_detection[j].z + out_detection[j].w / 2;
//
//        std::string type_name = "conf: " + conf;
//        text_dis_marker.text = type_name;
//        text_dis_marker.pose = pose;
//        //text_dis_marker_array.markers.push_back(text_dis_marker);
    }
    // std::cout << "marker_array_current.markers.size() " << marker_array_current.markers.size() << std::endl;
    //pub_marker_array_.publish(marker_array);

    visualization_msgs::Marker self;

    self.header.frame_id = "vehicle_link";
    self.header.stamp = ros::Time::now();
    self.ns = "box";
    self.id = 1;

    self.type = visualization_msgs::Marker::CUBE;
    self.action = visualization_msgs::Marker::ADD;

    self.pose.orientation.x = 0;
    self.pose.orientation.y = 0;
    self.pose.orientation.z = 0;
    self.pose.orientation.w = 1;


    self.scale.x = 12.94;
    self.scale.y = 7.97;

    self.scale.z = 7.12;
    self.color.r = 1;
    self.color.g = 1;
    self.color.b = 0;
    self.color.a = 1;

    self.pose.position.x = 12.94;
    self.pose.position.y = 0;
    self.pose.position.z = 6;

    self_array.markers.push_back(self);



    MessageManager::PublishVisualSelf(self_array);
    MessageManager::PublishVisualLidarObstacles(marker_array);


    // std::cout << "text_array_current.markers.size() " << text_array_current.markers.size() << std::endl;
    //pub_marker_text_.publish(text_dis_marker_array);

}
