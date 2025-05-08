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

#include "base/lane_base_segment.h"
#include "register/register.h"
#include "config/config.h"
#include "camera/segment/bisenet/trt_engine.h"
#include "monitor/log.h"

#include "inference/tensorrt/tensorrt_net.h"
#include "inference/inference_factory.h"

class BiSeNet : public LaneBaseSegment{
public:
    BiSeNet();
    ~BiSeNet();
    void Init(YAML::Node& config) override ;
    void Segment(cv::Mat &in_image, std::vector<int> &res) override ;
    void PreProcess(cv::Mat &in_image,  std::vector<float> &data) override ;

private:
   // std::unique_ptr<TrtEngine> engine_ptr_;
   // std::unique_ptr<TensorRTNet> engine_ptr_;
    std::unique_ptr<ConfigParser> config_;
    std::unique_ptr<Inference> inference_;

    const bool isBuildEngine = false;

    std::vector<std::string> input_names;
    std::vector<std::string> output_names;

};

//REG_CLASS(CameraLaneDetector);

REGISTER_CAMERA_SEGMENTATION(BiSeNet);
