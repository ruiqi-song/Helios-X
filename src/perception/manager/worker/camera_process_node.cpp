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

#include "camera_process_node.h"

#define DEVICE 0  // GPU id

bool CameraProcessWorkerNode::InitConfig(YAML::Node& config){
    camera_config = config;
    detector_id = camera_config["component"]["detector"]["name"].as<std::string>();
    tracker_id = camera_config["component"]["tracker"]["name"].as<std::string>();


    return true;
}

bool CameraProcessWorkerNode::InitMessage(std::string& name) {

    if(name == "FrontLongCamera")
    {
        MessageManager::AddFrontLongCameraCallback(&CameraProcessWorkerNode::ImgCallback, this);
    }
    else if(name == "FrontShortCamera")
    {
        MessageManager::AddFrontShortCameraCallback(&CameraProcessWorkerNode::ImgCallback, this);
    }
    else
    {
        MessageManager::AddImageRawCallback(&CameraProcessWorkerNode::ImgCallback, this);
    }

    std::string names = "FrontCamera";

    MessageManager::GetFuncByName(names);
    //MessageManager::LeftCameraCall_();



    //MessageManager::AddImageRawCallback(&CameraProcessWorkerNode::ImgCallback, this);

    return true;
}

bool CameraProcessWorkerNode::InitModule(){

    //detector_.reset(CameraBaseDetectorRegisterer::GetInstanceByName("Yolov5"));
    detector_.reset(CameraBaseDetectorRegisterer::GetInstanceByName(detector_id));
    detector_->Init(camera_config);
    //tracker_.reset(CameraBaseTrackerRegisterer::GetInstanceByName("DeepSorts"));
    tracker_.reset(CameraBaseTrackerRegisterer::GetInstanceByName(tracker_id));
    tracker_->Init(camera_config);
    return true;

}

bool CameraProcessWorkerNode::InitSharedData() {
    camera_obstalce_item_.reset(new CameraObstacle());


}

bool CameraProcessWorkerNode::InitWorkerNode(YAML::Node& config) {
    std::string name = config["node"].as<std::string>();
    InitConfig(config);
    InitModule();
    InitMessage(name);
    InitSharedData();
    return true;
}

bool CameraProcessWorkerNode::MessageToMat(const ImageRawAdapter::MessageType &msg, cv_bridge::CvImagePtr &cv_bridge_img, cv::Mat *img) {
    try {
        cv_bridge_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    ResizeMatImg(cv_bridge_img->image, *img);
    return true;
}


//void CameraProcessWorkerNode::ImgCallback(const sensor_msgs::Image & in_image_msg){
void CameraProcessWorkerNode::ImgCallback(const ImageRawAdapter::MessageType  &in_image_msg){
    cv_bridge::CvImagePtr cv_bridge_img;
    auto t_start = std::chrono::high_resolution_clock::now();

    //set cuda device
    cudaSetDevice(DEVICE);

    std::vector<CameraDetector::Detection> det_output;
    std::vector<DetectBox> trk_output;

    cv::Mat resized_img(CameraDetector::INPUT_H, CameraDetector::INPUT_W, CV_8UC3, cv::Scalar(128, 128, 128));

    // Message to Mat
    MessageToMat(in_image_msg, cv_bridge_img, &resized_img);

    cv::Mat img_;
    cv_bridge_img->image.copyTo(img_);
    CacheImageRaw(img_, in_image_msg.header.stamp.toSec());
    CacheImageViewer(cv_bridge_img->image, in_image_msg.header.stamp.toSec());

    //auto gray_image = cv_bridge_img->image.clone();

    //cv::cvtColor(gray_image, gray_image, cv::COLOR_BGR2GRAY);
    //MessageManager::PublishVisualCameraImage(*cv_bridge_img->toImageMsg());




    // Forward inference
    detector_->Detect(resized_img, det_output, batch_size);

    RestoreBox(cv_bridge_img->image, det_output);
    tracker_->Track(cv_bridge_img, det_output, trk_output);

    gLogInfo << det_output.size() << " <<<<<<>>>>>>" << trk_output.size() << std::endl;

    //show detection results
    cv_bridge_img->header.stamp = in_image_msg.header.stamp;
    PostProcess(cv_bridge_img, trk_output, out_objects);

//    auto trace = camera_shared_obstacle_->getAllMsg();
//    auto ob = camera_shared_obstacle_->getMsgNearestTime(in_image_msg.header.stamp);
//    gLogInfo << ">>>>>>>>>>>>>> " << trace.size() << std::endl;
//    for(int i = 0; i < trace.size(); i++){
//        for(int j = 0; j < trace[i]->data.objects.size(); j++){
////            if(trace[i]->data.objects.data()->label == "truck")
//           // {
//              if(trace[i]->data.objects[j].id == 1 || trace[i]->data.objects[j].id == 9 || trace[i]->data.objects[j].id == 10 ) {
//                    cv::circle(cv_bridge_img->image,
//                               cv::Point2f(trace[i]->data.objects[j].x + trace[i]->data.objects[j].width / 2,
//                                           trace[i]->data.objects[j].y +
//                                           trace[i]->data.objects[j].height / 2), 5, CV_RGB(255, 0, 0), -1, 1, 0);
//                }
//           // }
//
//
//        }
//    }

    MessageManager::PublishVisualCameraImage(*cv_bridge_img->toImageMsg());





    auto t_end = std::chrono::high_resolution_clock::now();
    float total = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    gLogInfo <<  "camera process node exec time: " << total << " ms." << std::endl;

}


void CameraProcessWorkerNode::RegistAllAlgorithms(){

    //ioc.RegisterType<CameraBaseDetector, Yolov5>("camera_obstalce_detector");
    //ioc.RegisterType<CameraBaseTracker, DeepSort>("camera_obstacle_tracker");

    RegisterFactoryYolov5();
    RegisterFactoryDeepSorts();
}

bool CameraProcessWorkerNode::ResizeMatImg(cv::Mat& img, cv::Mat& resized_img) {
    int w, h, x, y;
    float r_w = CameraDetector::INPUT_W / (img.cols*1.0);
    float r_h = CameraDetector::INPUT_H / (img.rows*1.0);
    if (r_h > r_w) {
        w = CameraDetector::INPUT_W;
        h = r_w * img.rows;
        x = 0;
        y = (CameraDetector::INPUT_H - h) / 2;
    } else {
        w = r_h * img.cols;
        h = CameraDetector::INPUT_H;
        x = (CameraDetector::INPUT_W - w) / 2;
        y = 0;
    }

    cv::Mat re(h, w, CV_8UC3);

    cv::resize(img, re, re.size(), 0, 0, cv::INTER_LINEAR);
    re.copyTo(resized_img(cv::Rect(x, y, re.cols, re.rows)));
    return true;
}


void CameraProcessWorkerNode::RestoreBox(cv::Mat &img, std::vector<CameraDetector::Detection> &res) {
    for (size_t i = 0; i < res.size(); ++i){
        auto r = get_rect(img, res[i].bbox);
        res[i].bbox[0] = r.x;
        res[i].bbox[1] = r.y;
        res[i].bbox[2] = r.width;
        res[i].bbox[3] = r.height;

    }
}


cv::Rect CameraProcessWorkerNode::get_rect(cv::Mat& img, float bbox[4]) {
    int l, r, t, b;
    float r_w = CameraDetector::INPUT_W / (img.cols * 1.0);
    float r_h = CameraDetector::INPUT_H / (img.rows * 1.0);
    if (r_h > r_w) {
        l = bbox[0] - bbox[2] / 2.f;
        r = bbox[0] + bbox[2] / 2.f;
        t = bbox[1] - bbox[3] / 2.f - (CameraDetector::INPUT_H - r_w * img.rows) / 2;
        b = bbox[1] + bbox[3] / 2.f - (CameraDetector::INPUT_H - r_w * img.rows) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - bbox[2] / 2.f - (CameraDetector::INPUT_W - r_h * img.cols) / 2;
        r = bbox[0] + bbox[2] / 2.f - (CameraDetector::INPUT_W - r_h * img.cols) / 2;
        t = bbox[1] - bbox[3] / 2.f;
        b = bbox[1] + bbox[3] / 2.f;
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }

    return cv::Rect(l, t, r - l, b - t);

}

void CameraProcessWorkerNode::CacheCameraObstacle(const CameraObstaclesAdapter::MessageType &objects,
                                                  const double &timestamp){
    camera_obstalce_item_.reset(new CameraObstacle());

    camera_obstalce_item_->data = objects;
    camera_obstalce_item_->timestamp = timestamp;
    camera_shared_obstacle_->add(camera_obstalce_item_);
}

void CameraProcessWorkerNode::CacheImageViewer(const cv::Mat &image,
                                             const double &timestamp) {
    camera_item_.reset(new CameraItem());

    camera_item_->data = image;
    camera_item_->timestamp = timestamp;
    camera_shared_data_->add(camera_item_);
}

void CameraProcessWorkerNode::CacheImageRaw(const cv::Mat &image,
                                            const double &timestamp) {
    camera_item_.reset(new CameraItem());

    camera_item_->data = image;
    camera_item_->timestamp = timestamp;
    camera_shared_raw_->add(camera_item_);
}



void CameraProcessWorkerNode::PostProcess(cv_bridge::CvImagePtr &cv_img, std::vector<DetectBox> &res,
                                          CameraObstaclesAdapter::MessageType & out_objects)
{
    out_objects.objects.clear();
    cv::Mat img = cv_img->image;
    out_objects.header.frame_id = "pylon_camera";
    out_objects.header.stamp = cv_img->header.stamp;
    for (size_t j = 0; j < res.size(); j++) {
        singleObject.x = res[j].x1;
        singleObject.y = res[j].y1;
        singleObject.width = res[j].x2 - res[j].x1;
        singleObject.height = res[j].y2 - res[j].y1;
        cv::Rect r = cv::Rect(singleObject.x, singleObject.y, singleObject.width, singleObject.height);

        singleObject.confidence = res[j].confidence;

        if((int) res[j].trackID >= 0){
            singleObject.id = (int) res[j].trackID;
        }
        else{
            singleObject.id = 0;
        }


        int class_id = (int) res[j].classID;

        switch (class_id) {
            case 0:
                cv::rectangle(img, r, cv::Scalar(0, 255, 255), 3);
                singleObject.label = "rock";
                singleObject.behavior_state = 1;
                singleObject.type = 0;
                break;
            case 1:
                cv::rectangle(img, r, cv::Scalar(132, 227, 255), 3);
                singleObject.label = "person";
                singleObject.behavior_state = 0;
                singleObject.type = 1;
                break;
            case 2:
                cv::rectangle(img, r, cv::Scalar(0, 0, 255), 3);
                singleObject.label = "lorry";
                singleObject.behavior_state = 0;
                singleObject.type = 2;
                break;
            case 3:
//                if(abs(singleObject.confidence - 0.904026) < 0.0001){
//                    break;
//                }
                cv::rectangle(img, r, cv::Scalar(0, 255, 0), 3);
                singleObject.label = "truck";
                singleObject.behavior_state = 0;
                singleObject.type = 3;
//                gLogInfo << "*****************" << singleObject.score << " " << singleObject.confidence << std::endl;
                break;
            case 7:
                cv::rectangle(img, r, cv::Scalar(0, 0, 85), 3);
                singleObject.label = "sign";
                singleObject.behavior_state = 1;
                singleObject.type = 4;
                break;
            case 5:
                cv::rectangle(img, r, cv::Scalar(42, 42, 128), 3);
                singleObject.label = "car";
                singleObject.behavior_state = 0;
                singleObject.type = 5;
                break;
            case 6:
                cv::rectangle(img, r, cv::Scalar(192, 192, 192), 3);
                singleObject.label = "auxiliary";
                singleObject.behavior_state = 0;
                singleObject.type = 6;
                break;
            case 4:
                cv::rectangle(img, r, cv::Scalar(100, 100, 192), 3);
                singleObject.label = "warning";
                singleObject.behavior_state = 1;
                singleObject.type = 7;
                break;
            case 8:
                cv::rectangle(img, r, cv::Scalar(89, 32, 200), 3);
                singleObject.label = "excavator";
                singleObject.behavior_state = 0;
                singleObject.type = 8;
                break;
        }
        singleObject.fusion_state = 1;
//        if(abs(singleObject.confidence - 0.904026) > 0.00001 && singleObject.label != "truck"){
//            out_objects.objects.push_back(singleObject);
//        }
        out_objects.objects.push_back(singleObject);


        if(class_id == 0 || class_id ==4 || class_id == 7 ){
            cv::putText(img,  singleObject.label, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_COMPLEX, 1,
                        cv::Scalar(0, 0, 0xFF), 2);
        }else{
            cv::putText(img,  singleObject.label + " id:" + std::to_string(singleObject.id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_COMPLEX, 1,
                        cv::Scalar(0, 0, 0xFF), 2);
        }

    }

    out_objects.header.stamp = cv_img->header.stamp;


    CacheCameraObstacle(out_objects, cv_img->header.stamp.toSec());



    //auto lidar_1 = lidar_shared_data_->getMsgBeforeTime(ros::Time().fromSec(camera_obstalce_item_->timestamp));
   // auto lidar_2 = lidar_shared_data_->getMsgNearestTime(ros::Time().fromSec(camera_obstalce_item_->timestamp));

    MessageManager::PublishCameraObstacles(out_objects);

}

