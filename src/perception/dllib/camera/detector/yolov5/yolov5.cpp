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
#include "yolov5.h"

Yolov5::Yolov5(){
    gLogInfo << "construct the Yolov5 object succesfully!" << std::endl;
}
Yolov5::~Yolov5(){
    gLogInfo << "delete the Yolov5 object succesfully" << std::endl;
}

void Yolov5::Init(YAML::Node& config){
    //const std::string engine_path = "/home/ricky/casias/modules/inference/tensorrt/build/detection.engine";
    //const std::string engine_path = "/home/ricky/casias/modules/inference/tensorrt/build/new.engine";
    //const std::string engine_path = "/home/ricky/casias/kittil.engine";
    const std::string engine_path = config["component"]["detector"]["model"].as<std::string>();
    std::string conf_path = config["component"]["detector"]["config"].as<std::string>();

    inference_.reset(CreateInferenceEngine("TensorRTNet", conf_path, engine_path,
                                           output_names, input_names, isBuildEngine));

    //tensorrt_net_ptr_.reset(new TensorRTNet(engine_path, isBuildEngine));
    config_.reset(new ConfigParser(conf_path));

    model_name = config_->context["model_name"];
    class_num = std::stoi(config_->context["class_num"]);
    input_h = std::stoi(config_->context["input_h"]);
    input_w = std::stoi(config_->context["input_w"]);
    ignore_thresh = std::stof(config_->context["ignore_thresh"]);
    max_output_bbox_count = std::stoi(config_->context["max_output_bbox_count"]);
    nms_thresh = std::stof(config_->context["nms_thresh"]);
    conf_thresh = std::stof(config_->context["conf_thresh"]);

    gLogInfo << "nms_thresh" << nms_thresh << "conf_thresh" << conf_thresh << std::endl;

}

void Yolov5::Detect(cv::Mat &input, std::vector<CameraDetector::Detection>& output, int& batchSize){
    static float data[3 * CameraDetector::INPUT_H * CameraDetector::INPUT_W];
    static float prob[CameraDetector::OUTPUT_SIZE];
    //std::vector<float> data_(3 * CameraDetector::INPUT_H * CameraDetector::INPUT_W);
    image2Vec(data, input);
    //tensorrt_net_ptr_->Infer(data, prob, batchSize);
    inference_->Infer(data, prob, batchSize, "data", "prob");

    nms(output, &prob[0], conf_thresh, nms_thresh);

}



float Yolov5::iou(float lbox[4], float rbox[4]) {
    float interBox[] = {
            std::max(lbox[0] - lbox[2]/2.f , rbox[0] - rbox[2]/2.f), //left
            std::min(lbox[0] + lbox[2]/2.f , rbox[0] + rbox[2]/2.f), //right
            std::max(lbox[1] - lbox[3]/2.f , rbox[1] - rbox[3]/2.f), //top
            std::min(lbox[1] + lbox[3]/2.f , rbox[1] + rbox[3]/2.f), //bottom
    };

    if(interBox[2] > interBox[3] || interBox[0] > interBox[1])
        return 0.0f;

    float interBoxS = (interBox[1] - interBox[0]) * (interBox[3] - interBox[2]);
    return interBoxS / (lbox[2] * lbox[3] + rbox[2] * rbox[3] - interBoxS);
}

bool Yolov5::cmp(const CameraDetector::Detection& a, const CameraDetector::Detection& b) {
    return a.conf > b.conf;
}


void Yolov5::nms(std::vector<CameraDetector::Detection>& res, float *output, float conf_thresh, float nms_thresh) {
    int det_size = sizeof(CameraDetector::Detection) / sizeof(float);

    std::map<float, std::vector<CameraDetector::Detection>> m;
    for (int i = 0; i < output[0] && i < CameraDetector::MAX_OUTPUT_BBOX_COUNT; i++) {
        if (output[1 + det_size * i + 4] <= conf_thresh) continue;
        CameraDetector::Detection det;
        memcpy(&det, &output[1 + det_size * i], det_size * sizeof(float));
        if (m.count(det.class_id) == 0) m.emplace(det.class_id, std::vector<CameraDetector::Detection>());
        m[det.class_id].push_back(det);
    }
    for (auto it = m.begin(); it != m.end(); it++) {
        //std::cout << it->second[0].class_id << " --- " << std::endl;
        auto& dets = it->second;
        std::sort(dets.begin(), dets.end(), cmp);
        for (size_t m = 0; m < dets.size(); ++m) {
            auto& item = dets[m];
            res.push_back(item);
            for (size_t n = m + 1; n < dets.size(); ++n) {
                if (iou(item.bbox, dets[n].bbox) > nms_thresh) {
                    dets.erase(dets.begin() + n);
                    --n;
                }
            }
        }
    }
}


