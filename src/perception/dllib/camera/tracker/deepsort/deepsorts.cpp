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

#include "deepsorts.h"

using std::vector;
//using namespace deepsorts;

static deepsort::Logger gLoggers;


DeepSorts::DeepSorts(){
    //allDetections.clear();
    //out.clear();
    gLogInfo << "Construct DeepSorts object" << std::endl;

}

DeepSorts::~DeepSorts(){
    std::cout << "delete the deepsorts" << std::endl;
}


void DeepSorts::Init(YAML::Node& config) {
    gLogInfo << "init the DeepSorts tracker" << std::endl;
    const std::string engine_path = config["component"]["tracker"]["model"].as<std::string>();
    std::string conf_path = config["component"]["tracker"]["config"].as<std::string>();
    DS = new DeepSort(engine_path, 128, 256, 0, &gLoggers);
    config_.reset(new ConfigParser(conf_path));


}

bool DeepSorts::SetState(const int &class_id) {
    if(class_id == 0 || class_id == 1 || class_id == 4 || class_id == 7){
        return 0;
    } else{
        return 1;
    }
}

void DeepSorts::Track(cv_bridge::CvImagePtr &in_img_raw, std::vector<CameraDetector::Detection> &dets, std::vector<DetectBox> &outputs){

    outputs.clear();
    //vector<DetectBox> Detections;
    DetectBox Objects;

    std::vector<DetectBox> track_outputs;



    for (size_t i = 0; i < dets.size(); i++) {
        auto object = dets[i];
        int c     = object.class_id;
        int x     = object.bbox[0];
        int y     = object.bbox[1];
        int w     = object.bbox[2];
        int h     = object.bbox[3];
        float con = object.conf;

        int state_id = SetState(c);

        if (state_id == 1){
            Objects = DetectBox(x, y, x+w, y+h, con, c);
            track_outputs.push_back(Objects);
        }else{
            Objects = DetectBox(x, y, x+w, y+h, con, c);
            outputs.push_back(Objects);
        }


    }
    gLogInfo << outputs.size() << "outputs-----------" << std::endl;
    gLogInfo << track_outputs.size() << "track_outputs-----------" << std::endl;
    DS->sort(in_img_raw->image, track_outputs);
    gLogInfo << track_outputs.size() << "-----------" << std::endl;

    for (int i = 0; i< track_outputs.size(); i++){
        outputs.push_back(track_outputs[i]);
    }
    gLogInfo << outputs.size() << "outputs-----------" << std::endl;

/*
    for (size_t i = 0; i < Detections.size(); ++i) {
        std::cout << Detections[i].x1 << " " << Detections[i].x2 << " " << Detections[i].y1 << " " << Detections[i].y2 << std::endl;
        std::cout << Detections[i].confidence << std::endl;
        std::cout << Detections[i].classID << std::endl;
       // std::cout << Detections[i].trackID << std::endl;
        outputs[i].bbox[0] = (Detections[i].x1 + Detections[i].x2) / 2;
        outputs[i].bbox[1] = (Detections[i].y1 + Detections[i].y2) / 2;
        outputs[i].bbox[2] = Detections[i].x2 - Detections[i].x1;
        outputs[i].bbox[3] = Detections[i].y2 - Detections[i].y1;
        outputs[i].conf = Detections[i].confidence;
        outputs[i].class_id = Detections[i].classID;
        //outputs[i].trackID = Detections[i].trackID;
        std::cout << outputs[i].bbox << std::endl;
        std::cout << outputs[i].conf << std::endl;
        std::cout << outputs[i].class_id << std::endl;
        //std::cout << outputs[i].trackID << std::endl;
    }

    std::cout << "///////////////////////:8" << std::endl;
*/


    //PostProcess(in_img_raw->image, mDetections, sDetections, out_track_detections);

}


//void DeepSorts::PostProcess(cv::Mat &img, vector<DetectBox> &mres, vector<DetectBox> &sres,
//                              waytous_msgs::DetectedObjectArray & out_objects)
//{  out_objects.objects.clear();
//    out_objects.header.frame_id = "pylon_camera";
//    for (size_t j = 0; j < mres.size(); j++) {
//        //float *p = (float *) &res[j];
//        cv::Rect r (mres[j].x1, mres[j].y1,mres[j].x2 - mres[j].x1,mres[j].y2 - mres[j].y1);
//
//        //push object to msg
//        //waytous_msgs::DetectedObject singleObject;
//        //singleObject.x = res[j].bbox[0];
//        //singleObject.y = res[j].bbox[1];
//        singleObject.x = r.x;
//
//        singleObject.y = r.y;
//
//
//        singleObject.width = r.width;
//        singleObject.height = r.height;
//
//        singleObject.confidence = mres[j].confidence;
//        //singleObject.id = (int) mres[j].trackID;
//
//        int class_id = (int) mres[j].classID;
//        switch (class_id) {
//            case 1:
//                cv::rectangle(img, r, cv::Scalar(132, 227, 255), 3);
//                singleObject.label = "person";
//                break;
//            case 2:
//                cv::rectangle(img, r, cv::Scalar(0, 0, 255), 3);
//                singleObject.label = "lorry";
//                break;
//            case 3:
//                cv::rectangle(img, r, cv::Scalar(0, 255, 0), 3);
//                singleObject.label = "truck";
//                break;
//            case 5:
//                cv::rectangle(img, r, cv::Scalar(42, 42, 128), 3);
//                singleObject.label = "car";
//                break;
//            case 6:
//                cv::rectangle(img, r, cv::Scalar(192, 192, 192), 3);
//                singleObject.label = "auxiliary";
//                break;
//            case 8:
//                cv::rectangle(img, r, cv::Scalar(89, 32, 200), 3);
//                singleObject.label = "excavator";
//                break;
//        }
//        out_objects.objects.push_back(singleObject);
//
//
//        cv::putText(img,  singleObject.label + " id:" + std::to_string(singleObject.id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_COMPLEX, 1,
//                    cv::Scalar(0, 0, 0xFF), 2.5);
//        //ROS_INFO("cols>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d",img.cols);
//        //ROS_INFO("rows>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d",img.rows);
//    }
//
//    for (size_t j = 0; j < sres.size(); j++) {
//        //float *p = (float *) &res[j];
//        cv::Rect r (sres[j].x1, sres[j].y1,sres[j].x2 - sres[j].x1,sres[j].y2 - sres[j].y1);
//
//        //push object to msg
//        //waytous_msgs::DetectedObject singleObject;
//        //singleObject.x = res[j].bbox[0];
//        //singleObject.y = res[j].bbox[1];
//        singleObject.x = r.x;
//
//        singleObject.y = r.y;
//
//
//        singleObject.width = r.width;
//        singleObject.height = r.height;
//
//        singleObject.confidence = sres[j].confidence;
//        //singleObject.id = (int) sres[j].trackID;
//
//        int class_id = (int) sres[j].classID;
//
//        switch (class_id) {
//            case 0:
//                cv::rectangle(img, r, cv::Scalar(0, 255, 255), 3);
//                singleObject.label = "rock";
//                break;
//            case 4:
//                cv::rectangle(img, r, cv::Scalar(0, 0, 85), 3);
//                singleObject.label = "sign";
//                break;
//            case 7:
//                cv::rectangle(img, r, cv::Scalar(100, 100, 192), 3);
//                singleObject.label = "warning";
//                break;
//        }
//        out_objects.objects.push_back(singleObject);
//
//
//        cv::putText(img, singleObject.label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_COMPLEX, 1,
//                    cv::Scalar(0, 0, 0xFF), 2.5);
//        //ROS_INFO("cols>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d",img.cols);
//        //ROS_INFO("rows>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d",img.rows);
//    }
//    //ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d",out_objects.objects.size());
//
//
//
//}