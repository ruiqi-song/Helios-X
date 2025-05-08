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


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "eigen3/Eigen/Dense"
#include <opencv2/core/eigen.hpp>
#include <opencv2/flann.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/PCLPointCloud2.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>

#include <Eigen/Core>
#include <Eigen/Geometry>


#include <jsk_recognition_utils/geo/cube.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "base/post_base_fusion.h"
#include "register/register.h"
#include "message/rosmsgs/obstacles/DetectedObjectArray.h"
#include "message/rosmsgs/obstacles/DetectedObject.h"

#include "object_ranging.h"

#include "config/config.h"

//#include "inference/tensorrt/common/logger.h"
#include "monitor/log.h"



class PostFusion : public PostBaseFusion{
public:
    PostFusion();
    ~PostFusion();
    void Init(tf::Transform &livox_vehicle, tf::Transform &camera_vehicle,
              cv::Mat camera_instrinsics, cv::Mat distortion_coefficients);
    void Fusion(
            waytous_msgs::DetectedObjectArray &camera_obstacles,
            waytous_msgs::DetectedObjectArray &lidar_obstacles,
            waytous_msgs::DetectedObjectArray &radar_obstacles,
            waytous_msgs::DetectedObjectArray &fusion_obstacles,
            cv::Mat &image_raw,
            sensor_msgs::PointCloud2 &point_cloud,
            cv::Mat &fusion_image);

private:

    waytous_msgs::DetectedObjectArray SetState(waytous_msgs::DetectedObjectArray &camera_obstacles,
            waytous_msgs::DetectedObjectArray &lidar_obstacles,
            cv::Mat &image_raw, cv::Mat &fusion_image);

    void MatchedFromCameraToLidar(waytous_msgs::DetectedObjectArray &camera_obstacles,
            waytous_msgs::DetectedObjectArray &obstacles3D_in_camera);

    void TransformRangeToVision(const waytous_msgs::DetectedObjectArray &in_range_detections,
                                                   waytous_msgs::DetectedObjectArray &out_in_cv_range_detections,
                                                   waytous_msgs::DetectedObjectArray &out_out_cv_range_detections);

    bool IsObjectInImage(const waytous_msgs::DetectedObject &in_detection);

    double GetDistanceToObject(const waytous_msgs::DetectedObject &in_object);
    cv::Rect ProjectDetectionToRect(const waytous_msgs::DetectedObject &in_detection);
    void CheckMinimumDimensions(waytous_msgs::DetectedObject &in_out_object);
    void InferObjectPose(waytous_msgs::DetectedObject &object, cv::Mat &fusion_image_);
    void FindTransform();
    cv::Point3f TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform);
    cv::Point2i ProjectPoint(const cv::Point3f &in_point);
    void PointCloudCallback(const sensor_msgs::PointCloud2& in_pointcloud_msg);

    bool GetTF(tf::StampedTransform& camera_vehicle_tf_, tf::StampedTransform& vehicle_lidar_tf_);




    waytous_msgs::DetectedObjectArray camera_matched_obstacles_;
    waytous_msgs::DetectedObjectArray camera_unmatched_obstacles_;
    waytous_msgs::DetectedObjectArray lidar_matched_obstacles_;
    waytous_msgs::DetectedObjectArray lidar_unmatched_obstacles_;

    cv::Mat image_;

    cv::Size image_size_;
    double overlap_threshold_ = 0.5;
    int query_num_ = 3;

    std::mutex valid_points_mutex_;
    bool image_ok_ = true;
    bool tf_ok_ = false;

    double car_width_, car_height_, car_depth_;
    double person_width_, person_height_, person_depth_;
    double truck_width_, truck_depth_, truck_height_;
    double rock_width_, rock_height_, rock_depth_;

    std::unordered_map<std::string, double> label_size_map;
    double objects_true_sizes[9];

    std::vector<cv::Point2f> valid_2d_points_; // 2d projected points
    std::vector<cv::Point3f> valid_3d_points_; // 3d points in fov of camera
    pcl::PointCloud<pcl::PointXYZI> point_cloud_;

    std::vector<cv::Point2f> projected_points_;
    std::vector<cv::Point3f> all_points_;

//    tf::Transform os_vehicle_;
//    tf::Transform livox_vehicle_;
//    tf::Transform os_camera_;
//
//    tf::Transform camera_livox_;
//
//    tf::Transform camera_vehicle_;
//    tf::Transform vehicle_lidar_;



    tf::StampedTransform camera_vehicle_tf_;
    tf::StampedTransform vehicle_lidar_tf_;


    cv::Mat camera_instrinsics_;
    cv::Mat distortion_coefficients_;
    float fx_;
    float fy_;
    float cx_;
    float cy_;

    //tf::TransformListener *transform_listener_;
    cv::Mat rvec_, tvec_;

    cv::flann::Index kdtree_;

    std::unordered_map<std::string, double> label_size_map_;
    cv::Mat fusion_image_;
    ObjectRanging ranging_;

    YAML::Node obstacles_config_;


};
//REG_CLASS(CameraObstacleDetector)

REGISTER_FUSION(PostFusion);

