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

#include <iostream>
#include "voxelrcnn.h"

VoxelRCNN::VoxelRCNN(){
    std::cout << "construct the VoxelRCNN object succesfully!" << std::endl;
}
VoxelRCNN::~VoxelRCNN(){
    std::cout << "delete the VoxelRCNN object succesfully" << std::endl;
}

void VoxelRCNN::Init(YAML::Node& config){
    std::cout << "init the obstacle detector VoxelRCNN" << std::endl;
}

void VoxelRCNN::Detect(const float* in_points_array, const int in_num_points, std::vector<float>& out_detections,  std::vector<Box3D>& predResult){
    std::cout << "detect the obstacle by VoxelRCNN" << std::endl;
}