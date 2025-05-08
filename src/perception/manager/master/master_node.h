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
#include <map>
#include <thread>

#include "yaml-cpp/yaml.h"

#include "container/ioc_container.h"
#include "thread/thread.h"
#include "config/config.h"
#include "monitor/log.h"

#include "worker/worker_node.h"
#include "worker/camera_process_node.h"
#include "worker/lidar_process_node.h"
#include "worker/synch_fusion_node.h"
#include "worker/lane_segment_node.h"
#include "worker/transform_process_node.h"
#include "worker/trajectory_predict_node.h"

#include "base/camera_base_detector.h"
#include "base/camera_base_tracker.h"
#include "base/lidar_base_detector.h"
#include "base/post_base_fusion.h"
#include "base/lane_base_segment.h"

#include "camera/detector/yolov5/yolov5.h"
#include "camera/segment/bisenet/bisenet.h"
#include "camera/tracker/deepsort/deepsorts.h"
#include "lidar/detector/pointpillars/point_pillars.h"
#include "fusion/post_fusion.h"


#include "interface/message_manager.h"

#include "3Dobjects/visualize_detected_objects.h"



class WorkerNode;
typedef std::map<std::string, std::unique_ptr<WorkerNode>> WorkerNodeMap;

using configs = YAML::Node;
using node = std::string;
//class MasterNode : public Thread{


class MasterNode{
public:
    MasterNode();
    virtual ~MasterNode();

    bool Init();
    void Stop();

protected:
    void Run();

    static WorkerNodeMap workernode_map_;


private:
    void Schedule();
    bool InitWorkerNodes(YAML::Node& config);
    bool InitConfig(YAML::Node config);
    void RegistAllClass();


    MessageManager *message_manager_;

    IocContainer ioc;

    std::shared_ptr<WorkerNode> front_long_camera_;
    std::shared_ptr<WorkerNode> front_short_camera_;
    std::shared_ptr<WorkerNode> lane_segment_;
    std::shared_ptr<WorkerNode> lidar_process_;
    std::shared_ptr<WorkerNode> fusion_process_;
    std::shared_ptr<WorkerNode> transform_process_;
    std::shared_ptr<WorkerNode> trajectory_process_;

    std::unique_ptr<VisualizeDetectedObjects> visual_process_;

    //std::map<std::string, std::shared_ptr<WorkerNode>> camera_process_map_;
    //std::map<std::string, std::shared_ptr<WorkerNode>> lidar_process_map_;
    //std::map<std::string, std::shared_ptr<WorkerNode>> radar_process_map_;

    std::map<configs, std::shared_ptr<WorkerNode>> camera_process_nodes;

    YAML::Node config;
    YAML::Node front_long_camera_config;
    YAML::Node front_short_camera_config;
    YAML::Node ouster64_config;
    YAML::Node front_lidar_config;
    YAML::Node back_lidar_config;


    //std::string path = "src/perception/conf/dag/park_scene.yaml";
    std::string path = "/home/ricky/waytous_server/src/perception/conf/dag/park_scene2.yaml";
    //std::string path_ = "src/perception/conf/dag/park_scene.yaml";




};


