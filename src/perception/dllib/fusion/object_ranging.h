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

#define __APP_NAME__ "object_ranging"

#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <jsk_recognition_utils/geo/cube.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/flann.hpp>

#include <yaml-cpp/yaml.h>
#include <unordered_map>

#include "message/rosmsgs/obstacles/DetectedObjectArray.h"
#include "message/rosmsgs/obstacles/DetectedObject.h"


class ObjectRanging 
{
  // // camera info
  // cv::Size image_size_;
  // float fx_, fy_, cx_, cy_;

  //  


  // (projected) point cloud
  std::vector<cv::Point2f> valid_2d_points_, left_2d_points_, right_2d_points_; // 2d projected points
  std::vector<cv::Point3f> valid_3d_points_, left_3d_points_, right_3d_points_; // 3d points in fov of camera

  // kd-tree    
  cv::flann::Index left_kdtree_;
  cv::flann::Index right_kdtree_;
  int query_num_ = 2;//用于设置返回邻近点的个数


public: 
  ObjectRanging();
  // void SetImageSize(const cv::Size& image_size);
	
  // functions for kd-tree position
  void BuildKdTree(const std::vector<cv::Point3f>& all_points,
  				   const std::vector<cv::Point2f>& projected_points,
  				   cv::Size image_size);
  cv::Point3f RectPosition(cv::Point2f p2_2d, cv::Point2f p0_2d, cv::Point3f p0_3d, cv::Point2f p1_2d, cv::Point3f p1_3d);
  void KdTreePosition(waytous_msgs::DetectedObject &object);

  // function for object size based position
  Eigen::Vector3d
  TransformPointEigen(const Eigen::Vector3d &in_point, 
  	                                 const tf::Transform &in_transform);
  void SizeRayPosition(waytous_msgs::DetectedObject &object,
							   float fx_, float fy_, float cx_, float cy_,
							   cv::Size image_size_,
							   tf::Transform vehicle_camera_tf,
							   std::unordered_map<std::string, double> label_size_map);
};

