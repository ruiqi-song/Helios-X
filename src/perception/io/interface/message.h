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

#include "adapter.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "message/rosmsgs/obstacles/DetectedObjectArray.h"

using ImageRawAdapter = AdapterMessage<sensor_msgs::Image>;
using FrontLongCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using FrontShortCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using FrontLeftCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using FrontRightCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using LeftLateralCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using RightLateralCameraAdapter = AdapterMessage<sensor_msgs::Image>;
using RearCameraAdapter = AdapterMessage<sensor_msgs::Image>;

using PointCloud2Adapter =  AdapterMessage<::sensor_msgs::PointCloud2>;
using Ouster64Adapter = AdapterMessage<::sensor_msgs::PointCloud2>;
using HorizonAdapter = AdapterMessage<::sensor_msgs::PointCloud2>;
using RSLidar16Adapter = AdapterMessage<::sensor_msgs::PointCloud2>;;

using FrontRadarAdapter = AdapterMessage<sensor_msgs::PointCloud2>;
using LeftRadarAdapter = AdapterMessage<sensor_msgs::PointCloud2>;
using RightRadarAdapter = AdapterMessage<sensor_msgs::PointCloud2>;
using RearRadarAdapter = AdapterMessage<sensor_msgs::PointCloud2>;

using GpsOdomAdapter = AdapterMessage<nav_msgs::Odometry>;
using SemanticPointCloud2Adapter = AdapterMessage<sensor_msgs::PointCloud2>;
using SemanticImageRawAdapter = AdapterMessage<sensor_msgs::Image>;

using PerceptionObstacleAdapter = AdapterMessage<waytous_msgs::DetectedObject>;
using CameraObstaclesAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;
using LidarObstaclesAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;
using RadarObstaclesAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;
using FusionObstaclesAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;
using LaneObstaclesAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;
using PredictTrajectoryAdapter = AdapterMessage<waytous_msgs::DetectedObjectArray>;

using VisualMarkerAdapter = AdapterMessage<visualization_msgs::Marker>;
using VisualCameraObstaclesAdapter = AdapterMessage<sensor_msgs::Image>;
using VisualLidarObstaclesAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualFusionObstaclesAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualHistoryPointsAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualHistoryTrajectoryAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualSelfAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualPredictTrajectoryAdapter = AdapterMessage<visualization_msgs::MarkerArray>;
using VisualSegmentProjectAdapter = AdapterMessage<sensor_msgs::Image>;
using VisualSegmentContourAdapter = AdapterMessage<sensor_msgs::Image>;
using VisualCameraImageAdapter = AdapterMessage<sensor_msgs::Image>;
using VisualFusionImageAdapter = AdapterMessage<sensor_msgs::Image>;

using SystemTimeAdapter = AdapterMessage<ros::Time>;

