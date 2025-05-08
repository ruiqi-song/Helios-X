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
#include "bisenet.h"

BiSeNet::BiSeNet(){
    gLogInfo << "construct the BiSeNet object succesfully!" << std::endl;
}

BiSeNet::~BiSeNet(){
    gLogInfo << "delete the BiSeNet object succesfully!" << std::endl;
}

void BiSeNet::Init(YAML::Node& config) {
    const std::string engine_path = config["model"].as<std::string>();
    std::string conf_path = config["config"].as<std::string>();
    //engine_ptr_.reset(new TrtEngine(engine_path));

    //std::string engine_path = "/home/ricky/waytous_server/src/perception/conf/dllib/camera/segment/bisenet/bisenet-mine-v1.trt";


    //engine_ptr_.reset(new TensorRTNet(engine_path, false));
    inference_.reset(CreateInferenceEngine("TensorRTNet", conf_path, engine_path, output_names, input_names, isBuildEngine));
    config_.reset(new ConfigParser(conf_path));

}

void BiSeNet::PreProcess(cv::Mat &in_image, std::vector<float> &data) {
    int iH = 512;
    int iW = 1024;
    std::array<float, 3> mean = {0.485f, 0.456f, 0.406f};
    std::array<float, 3> std = {0.229f, 0.224f, 0.225f};
    int channel(3);
    cv::Mat infer_image;
    cv::cvtColor(in_image, infer_image, cv::COLOR_BGR2RGB);
    cv::Mat img_float;
    infer_image.convertTo(img_float, CV_32FC3, 1. / 255.);
    // HWC TO CHW
    std::vector<cv::Mat> input_channels(channel);
    cv::split(img_float, input_channels);


    // normalize
//    std::vector<float> input_data(iH * iW * channel);
    auto datai = data.data();
    int channelLength = iH * iW;
    for (int i = 0; i < channel; ++i)
    {
        cv::Mat normed_channel = (input_channels[i] - mean[i]) / std[i];
        memcpy(datai, normed_channel.data, channelLength * sizeof(float));
        datai += channelLength;
    }

//    std::array<float, 3> mean = {0.485f, 0.456f, 0.406f};
//    std::array<float, 3> variance = {0.229f, 0.224f, 0.225f};
//
//    float scale = 1 / 255.f;
//    for(int i = 0; i < 3; ++i){
//        variance[i] = 1.f / variance[i];
//    }
//
//    for(int h = 0; h < LaneSegment::iH; ++h){
//        cv::Vec3b *p = in_image.ptr<cv::Vec3b>(h);
//        for(int w = 0; w < LaneSegment::iW; ++w){
//            for(int c = 0; c < 3; ++c){
//                int idx = LaneSegment::iH * LaneSegment::iW * (2 - c) + LaneSegment::iW * h + w;
//                data[idx] = (p[w][c] * scale -mean[c]) * variance[c];
//            }
//        }
//    }

}

void BiSeNet::Segment(cv::Mat &in_image, std::vector<int> &res) {

    std::vector<float> data(LaneSegment::iW * LaneSegment::iH * 3);

    PreProcess(in_image, data);

    //res = engine_ptr_->doInference(data);

    inference_->Infer(data, res, 1, "input_image", "preds");


}