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

#include <array>
#include <vector>
#include <unordered_map>

#include <sensor_msgs/point_cloud_conversion.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "eigen3/Eigen/Dense"
#include <opencv2/core/eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/tf.h>

//#include <Eigen/Core>
//#include <Eigen/Geometry>

#include <random>
#include <unistd.h>

#include "yaml-cpp/yaml.h"

#include "register/register.h"
#include "monitor/log.h"

#include "worker_node.h"
#include "base/lane_base_segment.h"
#include "camera/segment/bisenet/bisenet.h"

#include "interface/message_manager.h"
#include "interface/message_manager.h"
#include "base/lane_base_segment.h"
#include "cache/camera_shared_obstacle.h"

using namespace std;

namespace std {
    template <>
    class hash< cv::Point >{
    public :
        size_t operator()(const cv::Point &pixel_cloud ) const
        {
            return hash<std::string>()( std::to_string(pixel_cloud.x) + "|" + std::to_string(pixel_cloud.y) );
        }
    };
};



class LaneSegmentWorkerNode : public WorkerNode {
public:
    LaneSegmentWorkerNode() = default;

    ~LaneSegmentWorkerNode() = default;

private:

    bool InitConfig(YAML::Node& config);
    bool InitMessage();
    bool InitModule();
    bool InitWorkerNode(YAML::Node& config);

    void RegistAllAlgorithms() override;

    bool ResizeMatImg(cv::Mat& img);
    bool MessageToMat(const sensor_msgs::Image::ConstPtr &msg, cv_bridge::CvImagePtr &cv_bridge_img);
    bool MatToMessage(const cv::Mat &img, sensor_msgs::Image::ConstPtr *msg);

    void imageCallback(const sensor_msgs::Image::ConstPtr & in_image_msg);
    void PointCallback(const PointCloud2Adapter::MessageType& cloud_msg);

    void PostProcess(std::vector<int> &res, cv::Mat &in_image);
    void PostProcess_(std::vector<int> &res, cv::Mat &in_image);
    void ShowObastacles(cv::Mat &cv_img, const CameraObstaclesAdapter::MessageType &out_objects);
    std::vector<std::vector<uint8_t>> get_color_map();
    std::vector<std::vector<uint8_t>> get_color_map_();
    void GetContours(cv::Mat &img, cv::Mat &res);
    void CloudProject(const PointCloud2Adapter::MessageType &in_cloud_msg, cv::Mat& pred);

    std::unique_ptr<LaneBaseSegment> segment_;

    tf::StampedTransform camera_lidar_tf_;

    cv::Mat tvec;
    cv::Mat rvec;


    int image_height = 1200;
    int image_width = 1920;

    static constexpr int INPUT_H = 512;
    static constexpr int INPUT_W = 1024;

    bool map_;
    bool state_;

    YAML::Node lane_config;
    std::string segment_id;


};
