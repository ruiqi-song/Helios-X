/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Header.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "message/rosmsgs/obstacles/DetectedObject.h"
#include "message/rosmsgs/obstacles/DetectedObjectArray.h"

#define __APP_NAME__ "visualize_detected_objects"

class VisualizeDetectedObjects
{
private:
  const double arrow_height_;
  const double label_height_;
  const double object_max_linear_size_ = 50.;
  double object_speed_threshold_;
  double arrow_speed_threshold_;
  double marker_display_duration_;

  int marker_id_;

  std_msgs::ColorRGBA label_color_, box_color_, hull_color_, arrow_color_, centroid_color_, model_color_;

  std::string input_topic_, ros_namespace_;

  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_detected_objects_;

  ros::Publisher publisher_markers_;

  visualization_msgs::MarkerArray ObjectsToLabels(const waytous_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToArrows(const waytous_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToBoxes(const waytous_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToModels(const waytous_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToHulls(const waytous_msgs::DetectedObjectArray &in_objects);

  visualization_msgs::MarkerArray ObjectsToCentroids(const waytous_msgs::DetectedObjectArray &in_objects);

  std::string ColorToString(const std_msgs::ColorRGBA &in_color);

  void DetectedObjectsCallback(const waytous_msgs::DetectedObjectArray &in_objects);

  bool IsObjectValid(const waytous_msgs::DetectedObject &in_object);

  float CheckColor(double value);

  float CheckAlpha(double value);

  std_msgs::ColorRGBA ParseColor(const std::vector<double> &in_color);

public:
  VisualizeDetectedObjects();
};


