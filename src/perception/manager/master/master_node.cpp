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

#include "master_node.h"

MasterNode::MasterNode(){};
MasterNode:: ~MasterNode(){};

bool MasterNode::Init() {
    gLogInfo << "init the TaskManager: " << std::endl;
    MessageManager::Init();
    gLogInfo << "init the TaskManager end: " << std::endl;
    read_service_config(path, config);
    gLogInfo << "init the TaskManager1: " << std::endl;
    InitConfig(config);
    gLogInfo << "init the TaskManager2: " << std::endl;
    RegistAllClass();
    InitWorkerNodes(config);
    Run();
    gLogInfo << "TaskManager is ready: " << std::endl;


    return true;
}

bool MasterNode::InitConfig(YAML::Node config) {
    front_long_camera_config = config["DAGstreaming"]["perception"]["camera"]["FrontLongCamera"];
    front_short_camera_config = config["DAGstreaming"]["perception"]["camera"]["FrontShortCamera"];
    ouster64_config = config["DAGstreaming"]["perception"]["lidar"]["Ouster64"];
    return true;
}

//bool MasterNode::InitConfig()
//{
//    read_service_config(path, config);
//    YAML::Node camera_config = config["DAGstreaming"]["perception"]["camera"];
//    YAML::Node::const_iterator n_it = camera_config.begin();
//
//    for (; n_it != camera_config.end(); n_it++){
//        std::string device_id = n_it->first.as<std::string>();
//        camera_process_nodes.insert(std::make_pair(camera_config[device_id], camera_process_));
//        gLogInfo << "camera process nodes map"  << std::endl;
//    }
//
//
//
//    return true;
//}


void MasterNode::Stop() {
    gLogInfo << "stop the TaskManager" << std::endl;
}
void MasterNode::Schedule() {
    /*
    for(auto& pair: subnode_map_){
        pair.second->Start();
    }

    std::cout << "Master start to schedule" << std::endl;

    for(auto& pair: subnode_map_){
        pair.second->Join();
    }
     */
    //std::cout<< "the thread id: "<< std::this_thread::get_id() << std::endl;
    //std::cout<< "the node thread id: "<< std::this_thread::get_id() << std::endl;

   // std::cout << "Master schedule exit" << std::endl;


}

bool MasterNode::InitWorkerNodes(YAML::Node& config){
    /*
    Subnode* cameraNode = new CameraProcessSubnode();
    Subnode* lidarNode = new LidarProcessSubnode();
    Subnode* fusionNode = new FusionProcessSubnode();

    subnode_map_.emplace("cameraNode", std::unique_ptr<Subnode>(cameraNode));
    subnode_map_.emplace("lidarNode", std::unique_ptr<Subnode>(lidarNode));
    subnode_map_.emplace("fusionNode", std::unique_ptr<Subnode>(fusionNode));
    */
    transform_process_ = ioc.ResolveShared<WorkerNode>("transform_process_subnode");
    transform_process_->RegistAllAlgorithms();
    transform_process_->InitWorkerNode(config);

//    for(auto iter = camera_process_nodes.begin(); iter != camera_process_nodes.end(); iter++) {
//        cout << iter->first << " : " << iter->second << endl;
//    }

    front_long_camera_ = ioc.ResolveShared<WorkerNode>("camera_process_subnode");
    front_long_camera_->RegistAllAlgorithms();
    front_long_camera_->InitWorkerNode(front_long_camera_config);
//
//    front_short_camera_ = ioc.ResolveShared<WorkerNode>("camera_process_subnode");
//    front_short_camera_->RegistAllAlgorithms();
//    front_short_camera_->InitWorkerNode(front_short_camera_config);

    lidar_process_ = ioc.ResolveShared<WorkerNode>("lidar_process_subnode");
    lidar_process_->RegistAllAlgorithms();
    lidar_process_->InitWorkerNode(ouster64_config);

    fusion_process_ = ioc.ResolveShared<WorkerNode>("fusion_process_subnode");
    fusion_process_->RegistAllAlgorithms();
    fusion_process_->InitWorkerNode(config);

    lane_segment_ = ioc.ResolveShared<WorkerNode>("lane_segment_subnode");
    lane_segment_->RegistAllAlgorithms();
    lane_segment_->InitWorkerNode(front_long_camera_config);

    trajectory_process_ = ioc.ResolveShared<WorkerNode>("trajectory_process_subnode");
    trajectory_process_->RegistAllAlgorithms();
    trajectory_process_->InitWorkerNode(config);

    //visual_process_.reset(new VisualizeDetectedObjects());



    return true;

}

void MasterNode::RegistAllClass(){
    ioc.RegisterType<WorkerNode, TransformProcessNode>("transform_process_subnode");
    ioc.RegisterType<WorkerNode, CameraProcessWorkerNode>("camera_process_subnode");
    ioc.RegisterType<WorkerNode, LidarProcessWorkerNode>("lidar_process_subnode");
    ioc.RegisterType<WorkerNode, FusionProcessWorkerNode>("fusion_process_subnode");
    ioc.RegisterType<WorkerNode, LaneSegmentWorkerNode>("lane_segment_subnode");
    ioc.RegisterType<WorkerNode, TrajectoryPredictNode>("trajectory_process_subnode");
}




void MasterNode::Run() { Schedule(); }