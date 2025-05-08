
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

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <fstream>
#include <iostream>

#include <vector>

#include "google/protobuf/text_format.h"
#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "yaml-cpp/yaml.h"

#include <fcntl.h>




/*
typedef struct Params{
    std::string camera_detector_ = "Yolov5";
    std::string camera_tracker_ = "DeepSort";
    std::string lidar_detector_ = "VoxelRCNN";
    std::string lidar_tracker_ = "Kalman";
    std::string lane_detector_ = "BiseNet";
    std::string fusion_ = "PostFusion";
};

Params params{
    camera_detector_("Yolov5");
        std::string camera_tracker_ = "DeepSort";
        std::string lidar_detector_ = "VoxelRCNN";
        std::string lidar_tracker_ = "Kalman";
        std::string lane_detector_ = "BiseNet";
        std::string fusion_ = "PostFusion";

};
 */

inline tf::Transform read_extrinsics(const std::string &link_name);
tf::Transform read_extrinsics(const std::string &link_name){
    tf::Transform transform;
    tf::Quaternion quat;

    YAML::Node config = YAML::LoadFile("/home/ricky/waytous_server/src/perception/conf/calibration/extrinsics/truck_urdf_shenbao_2021.yaml");
    auto node_link = config[link_name];

    transform.setOrigin(tf::Vector3(node_link["transform"]["translation"]["x"].as<float>(),
                                node_link["transform"]["translation"]["y"].as<float>(),
                                node_link["transform"]["translation"]["z"].as<float>()));

    quat.setRPY(node_link["transform"]["rotation"]["r"].as<float>(),
             node_link["transform"]["rotation"]["p"].as<float>(),
             node_link["transform"]["rotation"]["y"].as<float>());
    transform.setRotation(quat);

    return transform;

}

inline cv::Mat get_intrinsics();
cv::Mat get_intrinsics(){

    YAML::Node config = YAML::LoadFile("/home/ricky/waytous_server/src/perception/conf/calibration/instrinsics/camera_instrinsics.yaml");
    cv::Mat camera_matrix;
    camera_matrix = cv::Mat(3, 3, CV_64F);

    auto camera_matrix_ = config["camera_matrix"]["data"];
    auto distortion_coefficients_ = config["distortion_coefficients"]["data"];

    for(int i = 0; i < camera_matrix_.size(); i++){
        camera_matrix.at<double>(i) = camera_matrix_[i].as<double>();
        std::cout << "************************* " << camera_matrix.at<double>(i) << std::endl;
    }

    return camera_matrix;

}

inline cv::Mat get_distortion();
cv::Mat get_distortion(){

    YAML::Node config = YAML::LoadFile("/home/ricky/waytous_server/src/perception/conf/calibration/instrinsics/camera_instrinsics.yaml");
    cv::Mat distortion_coefficients;

    distortion_coefficients = cv::Mat(5, 1, CV_64F);
    auto distortion_coefficients_ = config["distortion_coefficients"]["data"];

    for(int j = 0; j < distortion_coefficients_.size(); j++){
        distortion_coefficients.at<double>(j) = distortion_coefficients_[j].as<double>();
        std::cout << "+++++++++++++++++++++++ " << distortion_coefficients.at<double>(j) << std::endl;
    }

    return distortion_coefficients;

}

inline void read_service_config(std::string& path, YAML::Node &config);
void read_service_config(std::string& path, YAML::Node &config){
    //path = "src/perception/conf/dag/mine_scene.yaml";
    config = YAML::LoadFile(path);
}



/**
 * @brief Parses the content of the config file of dllib
 */
class ConfigParser{
public:
    ConfigParser(std::string& file)
    : size_(10),
      file_(file)
      {
        FileStream();
      }

    std::map<std::string, std::string> context;

private:

    void FileStream(){
        std::ifstream file;
        file.open(file_.data());
        assert(file.is_open());

        std::string s;
        while(getline(file,s))
        {
            Split(s, vec,":");
            context.insert(std::make_pair(vec[0], Strip(vec[1])));
            vec.clear();
        }
        file.close();
    }

    void Split(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiter)
    {
        std::string::size_type lastPos = str.find_first_not_of(delimiter, 0);
        std::string::size_type pos = str.find_first_of(delimiter, lastPos);
        while(std::string::npos != pos || std::string::npos != lastPos)
        {
            tokens.push_back(str.substr(lastPos, pos - lastPos));

            lastPos = str.find_first_not_of(delimiter, pos);
            pos = str.find_first_of(delimiter, lastPos);
        }
    }

    std::string& Strip(std::string& str)
    {
        str.erase(0,str.find_first_not_of(" "));
        str.erase(str.find_last_not_of(" ") + 1);
        return str;
    }

    int size_;
    std::string file_;
    std::vector<std::string> vec;

};



