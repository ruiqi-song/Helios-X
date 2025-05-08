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

#include "object_ranging.h"

ObjectRanging::ObjectRanging()
{

	
}

// void
// ObjectRanging::SetImageSize(const cv::Size& image_size)
// {
// 	image_size = image_size;
// }
void ObjectRanging::BuildKdTree(const std::vector<cv::Point3f>& all_points,
  						  const std::vector<cv::Point2f>& projected_points,
  						  cv::Size image_size)
{

	using namespace std::chrono;

	steady_clock::time_point t1 = steady_clock::now();

	// std::vector<cv::Point2f> valid_2d_points = { { 1,1 },{ 2, 2},{ 3, 3},{ 4, 4},{ 2, 4} };

	// update (projected) point clouds
	valid_2d_points_.clear();
	valid_3d_points_.clear();
	left_2d_points_.clear();
	left_3d_points_.clear();
	right_2d_points_.clear();
	right_3d_points_.clear();

	for(size_t i = 0; i < projected_points.size(); i++) {        
	cv::Point2f p = projected_points[i];
	if(isnan(p.x) or isnan(p.y))
	  continue;
	if (!(p.x >= 0 && p.x < image_size.width && p.y >= 0 && p.y < image_size.height))
	  continue; 

	valid_2d_points_.push_back(p);
	valid_3d_points_.push_back(all_points[i]);

	if(p.x < image_size.width/2) {
	  left_2d_points_.push_back(p);
	  left_3d_points_.push_back(all_points[i]);
	} else {
	  right_2d_points_.push_back(p);
	  right_3d_points_.push_back(all_points[i]);
	}


	}
//	std::cout << "valid_2d_points size: " << valid_2d_points_.size() <<std::endl;
//	std::cout << "left_2d_points size: " << left_2d_points_.size() <<std::endl;
//	std::cout << "right_2d_points size: " << right_2d_points_.size() <<std::endl;

	if(left_2d_points_.size() > 10) {
	cv::Mat source= cv::Mat(left_2d_points_).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::KDTreeIndexParams index_params(2); 
	// cv::flann::Index kdtree(source, index_params);
	// kdtree_ = kdtree.clone();
	left_kdtree_.build(source, index_params);
	steady_clock::time_point t2 = steady_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);  

	}
	if(right_2d_points_.size() > 10) {
	cv::Mat source= cv::Mat(right_2d_points_).reshape(1);
	source.convertTo(source, CV_32F);
	cv::flann::KDTreeIndexParams index_params(2); 
	// cv::flann::Index kdtree(source, index_params);
	// kdtree_ = kdtree.clone();
	right_kdtree_.build(source, index_params);
	steady_clock::time_point t2 = steady_clock::now();
	duration<double> time_span = duration_cast<duration<double>>(t2 - t1);  

	}
}


cv::Point3f ObjectRanging::RectPosition(cv::Point2f p2_2d, cv::Point2f p0_2d, cv::Point3f p0_3d, cv::Point2f p1_2d, cv::Point3f p1_3d)
{
  cv::Point3f p2_3d; // return
  float X_v= (p1_3d.x - p0_3d.x) / (p1_2d.y - p0_2d.y);
  float Y_u= (p1_3d.y - p0_3d.y) / (p1_2d.x - p0_2d.x);
  p2_3d.x = p0_3d.x + X_v*(p2_2d.y - p0_2d.y);
  p2_3d.y = p0_3d.y + Y_u*(p2_2d.x - p0_2d.x);
  p2_3d.z = (p0_3d.z + p1_3d.z) / 2;
  return p2_3d;
}

void ObjectRanging::KdTreePosition(waytous_msgs::DetectedObject &object)
{
	// if both in-bbox and near-bbox points unavailable 

	std::vector<float> vec_query(2);//存放查询点的容器
	vec_query = {object.x+object.width/2, object.y+object.height/2};
	cv::flann::SearchParams search_params(32);
	// left
	std::vector<int> left_vec_index(query_num_);//存放返回的点索引 
	std::vector<float> left_vec_dist(query_num_);//存放距离
	left_kdtree_.knnSearch(vec_query, left_vec_index, left_vec_dist, query_num_, search_params);
	// right
	std::vector<int> right_vec_index(query_num_);//存放返回的点索引 
	std::vector<float> right_vec_dist(query_num_);//存放距离
	right_kdtree_.knnSearch(vec_query, right_vec_index, right_vec_dist, query_num_, search_params);

	cv::Point2f lp2d_0 = left_2d_points_[left_vec_index[0]]; 
	cv::Point3f lp3d_0 = left_3d_points_[left_vec_index[0]]; 
	cv::Point2f lp2d_1 = left_2d_points_[left_vec_index[1]]; 
	cv::Point3f lp3d_1 = left_3d_points_[left_vec_index[1]]; 

	cv::Point2f rp2d_0 = right_2d_points_[right_vec_index[0]]; 
	cv::Point3f rp3d_0 = right_3d_points_[right_vec_index[0]]; 
	cv::Point2f rp2d_1 = right_2d_points_[right_vec_index[1]]; 
	cv::Point3f rp3d_1 = right_3d_points_[right_vec_index[1]]; 


	cv::Point3f position;
	// use both left points
	if ( vec_query[0] < lp2d_0.x || vec_query[0] < lp2d_1.x) {
	  position = RectPosition(cv::Point2f(vec_query[0],vec_query[1]), lp2d_0, lp3d_0,
	                                   lp2d_1, lp3d_1);

	}
	// use both right points
	else if ( vec_query[0] > rp2d_0.x || vec_query[0] > rp2d_1.x) {
	  position = RectPosition(cv::Point2f(vec_query[0],vec_query[1]), rp2d_0, rp3d_0,
	                                   rp2d_1, rp3d_1);
	} else {
	 // use both side points    

	  position = RectPosition(cv::Point2f(vec_query[0],vec_query[1]),
	                                       lp2d_0.x > lp2d_1.x ? lp2d_0 : lp2d_1, lp2d_0.x > lp2d_1.x ? lp3d_0 : lp3d_1,
	                                       rp2d_0.x < rp2d_1.x ? rp2d_0 : rp2d_1, rp2d_0.x < rp2d_1.x ? rp3d_0 : rp3d_1);
	  
	}
	object.pose.position.x = position.x; 
	object.pose.position.y = position.y;
	object.pose.position.z = position.z;      


}


Eigen::Vector3d ObjectRanging::TransformPointEigen(const Eigen::Vector3d &in_point, const tf::Transform &in_transform)
{
  tf::Vector3 tf_point(in_point[0], in_point[1], in_point[2]);
  tf::Vector3 tf_point_t = in_transform * tf_point;
  // ROS_INFO("lidar point: %f %f %f",in_point.x, in_point.y, in_point.z);
  // ROS_INFO("camera point: %f %f %f",tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
  return Eigen::Vector3d(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}


void ObjectRanging::SizeRayPosition(waytous_msgs::DetectedObject &object,
							   float fx_, float fy_, float cx_, float cy_,
							   cv::Size image_size_,
							   tf::Transform vehicle_camera_tf,
							   std::unordered_map<std::string, double> label_size_map)
{    

    /* based on object size prior */   

    // ray direction
    Eigen::Vector3d dir_c_;
    Eigen::Vector3d norm_p(object.x + object.width/2, object.y+object.height/2,1); // object center point
    dir_c_[0] = (norm_p[0] - cx_) / fx_; // ; // X/Z
    dir_c_[1] = (norm_p[1] - cy_) / fy_; // ; // Y/Z
    dir_c_[2] = 1.0;

    // ROS_INFO_STREAM("object label: " <<  object.label);
    double object_true_size = label_size_map[object.label];
    double ff =  std::sqrt(std::pow(fy_, 2) + std::pow(object.x + object.width/2 - image_size_.width/2, 2));
    double depth = ff * object_true_size / object.height; 
    Eigen::Vector3d pos_c = depth * dir_c_; // position in camera frame

    Eigen::Vector3d pos_l = TransformPointEigen(pos_c, vehicle_camera_tf);


    // //ROS_INFO("ranging: %f %f", fy_, object.height);
    // ROS_INFO_STREAM("object_true_size: " << object_true_size);    

    // ROS_INFO("X : %f", pos_l[0]);
    // ROS_INFO("Y : %f", pos_l[1]);
    // ROS_INFO("Z : %f", pos_l[2]);
    object.pose.position.x = pos_l[0];
    object.pose.position.y = pos_l[1];
    object.pose.position.z = pos_l[2];
}