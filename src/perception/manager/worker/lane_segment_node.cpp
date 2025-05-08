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

#include "lane_segment_node.h"

bool LaneSegmentWorkerNode::InitConfig(YAML::Node& config) {

    camera_lidar_tf_.setData(camera_livox_);


    tvec = (cv::Mat_<float>(3,1) << camera_lidar_tf_.getOrigin().getX(),
            camera_lidar_tf_.getOrigin().getY(),camera_lidar_tf_.getOrigin().getZ());

    double angle = camera_lidar_tf_.getRotation().getAngle();
    tf::Vector3 axis = camera_lidar_tf_.getRotation().getAxis().normalize();
    rvec = (cv::Mat_<float>(3,1) << angle*axis.getX(), angle*axis.getY(), angle*axis.getZ());

    lane_config = config["component"]["segment"];
    segment_id = lane_config["name"].as<std::string>();



}


bool LaneSegmentWorkerNode::InitMessage() {
    MessageManager::AddHorizonCallback(&LaneSegmentWorkerNode::PointCallback, this);
    return true;
}

bool LaneSegmentWorkerNode::InitModule() {
    segment_.reset(LaneBaseSegmentRegisterer::GetInstanceByName(segment_id));
    segment_->Init(lane_config);

}

bool LaneSegmentWorkerNode::InitWorkerNode(YAML::Node& config) {
    map_ = false;
    state_ = true;
    InitConfig(config);
    InitModule();
    InitMessage();
}

void LaneSegmentWorkerNode::RegistAllAlgorithms(){

    //ioc.RegisterType<CameraBaseDetector, Yolov5>("camera_obstalce_detector");
    //ioc.RegisterType<CameraBaseTracker, DeepSort>("camera_obstacle_tracker");

    RegisterFactoryBiSeNet();

}

void LaneSegmentWorkerNode::PointCallback(const PointCloud2Adapter::MessageType &cloud_msg)
{
    if(state_){
        sleep(1);
        state_ = false;
    }


    auto in_image_item = camera_shared_raw_->getMsgNearestTime(cloud_msg.header.stamp);



    cv::Mat in_image = in_image_item->data;

    sensor_msgs::ImagePtr msgs = cv_bridge::CvImage(std_msgs::Header(), "bgr8", in_image).toImageMsg();
    MessageManager::PublishSemanticImageRaw(*msgs);
    //cv::cvtColor(in_image, in_image, cv::COLOR_BGR2RGB);



    int orgH{in_image.rows}, orgW{in_image.cols};

    ResizeMatImg(in_image);


    vector<int> res(INPUT_H * INPUT_W);

    auto t_start = std::chrono::high_resolution_clock::now();

    segment_->Segment(in_image, res);

    auto t_end = std::chrono::high_resolution_clock::now();
    float total = std::chrono::duration<float, std::milli>(t_end- t_start).count();
    gLogInfo <<  "lane segment node exec time: " << total << " ms." << std::endl;

    cv::Mat pred(cv::Size(LaneSegment::oW, LaneSegment::oH), CV_8UC3);

    cv::Mat pred_pub = in_image;
    PostProcess(res, pred);
    PostProcess_(res,pred_pub);
    cv::resize(pred, pred, cv::Size(1920, 1200), cv::INTER_CUBIC);
    cv::resize(pred_pub, pred_pub, cv::Size(1920, 1200), cv::INTER_CUBIC);


    CloudProject(cloud_msg, pred);

    //get obstacles
    auto obstacles_msg = camera_shared_obstacle_->getMsgNearestTime(cloud_msg.header.stamp);
    ShowObastacles(pred_pub, obstacles_msg->data);


    //postProcess(in_image_ptr->image, preds);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pred_pub).toImageMsg();


    //postProcess(in_image_ptr->image, pred);

    //pub_image_contour_.publish(in_image_ptr->toImageMsg());
    //pub_image_seg_.publish(*msg);

//    MessageManager::PublishSemanticImageRaw(*msgs);


}

void LaneSegmentWorkerNode::imageCallback(const sensor_msgs::Image::ConstPtr &in_image_msg)
{


    cv_bridge::CvImagePtr in_image_ptr_;

    // Message to Mat
    MessageToMat(in_image_msg, in_image_ptr_);

    cv::Mat in_image = in_image_ptr_->image;
    int orgH{in_image.rows}, orgW{in_image.cols};

    ResizeMatImg(in_image);

    vector<int> res;
    auto t_start = std::chrono::high_resolution_clock::now();
    segment_->Segment(in_image, res);
    auto t_end = std::chrono::high_resolution_clock::now();
    float total = std::chrono::duration<float, std::milli>(t_end- t_start).count();
    gLogInfo <<  "lane segment node exec time: " << total << " ms." << std::endl;

    cv::Mat pred(cv::Size(LaneSegment::oW, LaneSegment::oH), CV_8UC3);

    cv::Mat pred_pub = in_image;

    PostProcess_(res, pred);
    PostProcess_(res,pred_pub);
    cv::resize(pred, pred, cv::Size(1920, 1200), cv::INTER_CUBIC);
    cv::resize(pred_pub, pred_pub, cv::Size(1920, 1200), cv::INTER_CUBIC);

    auto cloud_msg = lidar_shared_data_->getMsgNearestTime(in_image_msg->header.stamp);

    CloudProject(cloud_msg->data, pred);

    //get obstacles
    auto obstacles_msg = camera_shared_obstacle_->getMsgNearestTime(in_image_msg->header.stamp);
    ShowObastacles(pred_pub, obstacles_msg->data);

    //postProcess(in_image_ptr->image, preds);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pred_pub).toImageMsg();



    //postProcess(in_image_ptr->image, pred);

    //pub_image_contour_.publish(in_image_ptr->toImageMsg());
    //pub_image_seg_.publish(*msg);
    MessageManager::PublishSemanticImageRaw(*msg);


}

bool LaneSegmentWorkerNode::ResizeMatImg(cv::Mat &img) {
    cv::resize(img, img, cv::Size(LaneSegment::iW, LaneSegment::iH), cv::INTER_CUBIC);
}

bool LaneSegmentWorkerNode::MessageToMat(const sensor_msgs::Image::ConstPtr &msg, cv_bridge::CvImagePtr &cv_bridge_img) {
    try {
        cv_bridge_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception & e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    return true;

}

bool LaneSegmentWorkerNode::MatToMessage(const cv::Mat &img, sensor_msgs::Image::ConstPtr *msg) {

}

void LaneSegmentWorkerNode::PostProcess_(vector<int> &res, cv::Mat &pred){
    // generate colored out

    std::vector<std::vector<uint8_t>> color_map = get_color_map_();

    int idx{0};

    for (int i{0}; i < LaneSegment::oH; ++i) {
        uint8_t *ptr = pred.ptr<uint8_t>(i);
        for (int j{0}; j < LaneSegment::oW; ++j) {
            if(res[idx] != 0) {
                ptr[0] = color_map[res[idx]][0];
                ptr[1] = color_map[res[idx]][1];
                ptr[2] = color_map[res[idx]][2];
            }
            //std::cout << "color}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}: "<<res[idx] << std::endl;
            ptr += 3;
            ++ idx;
        }
    }


};

void LaneSegmentWorkerNode::PostProcess(vector<int> &res, cv::Mat &pred){
    // generate colored out

    std::vector<std::vector<uint8_t>> color_map = get_color_map_();

    int idx{0};

    for (int i{0}; i < LaneSegment::oH; ++i) {
        uint8_t *ptr = pred.ptr<uint8_t>(i);
        for (int j{0}; j < LaneSegment::oW; ++j) {
            ptr[0] = color_map[res[idx]][0];
            ptr[1] = color_map[res[idx]][1];
            ptr[2] = color_map[res[idx]][2];

           // std::cout << "color}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}: "<<res[idx] << std::endl;
            ptr += 3;
            ++ idx;
        }
    }


};

void LaneSegmentWorkerNode::GetContours(cv::Mat &img, cv::Mat &res) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    findContours(res, contours, hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE,cv::Point());


    /*
    for(int i = 0; i < contours.size(); i++){
        for(int j=0; j<contours[i].size();j++){
            Point P = Point(contours[i][j].x, contours[i][j].y);
            contours
        }
    }
     */
    for(int i = 0; i < contours.size(); i++){
        drawContours(img,contours,i,cv::Scalar(0,0,255),3,8,hierarchy);
    }

}


std::vector<std::vector<uint8_t>> LaneSegmentWorkerNode::get_color_map() {
    vector<vector<uint8_t>> color_map(256, vector<uint8_t>(3));
    std::minstd_rand rand_eng(123); //150
    std::uniform_int_distribution<uint8_t> u(0, 255);
    for (int i{0}; i < 256; ++i) {
        for (int j{0}; j < 3; ++j) {
            color_map[i][j] = u(rand_eng);
        }
    }
    return color_map;
}

std::vector<std::vector<uint8_t>> LaneSegmentWorkerNode::get_color_map_(){
    vector<vector<uint8_t>> color_map(12, vector<uint8_t>(3));

//    color_map[0][0] = 0;
//    color_map[0][1] = 0;
//    color_map[0][2] = 0;
//
//    color_map[1][0] = 209;
//    color_map[1][1] = 206;
//    color_map[1][2] = 1;
//
//    color_map[2][0] = 1;
//    color_map[2][1] = 255;
//    color_map[2][2] = 255;
//
//    color_map[3][0] = 1;
//    color_map[3][1] = 1;
//    color_map[3][2] = 255;
//
//    color_map[4][0] = 128;
//    color_map[4][1] = 128;
//    color_map[4][2] = 240;


//map
//    color_map[0][0] = 113;
//    color_map[0][1] = 179;
//    color_map[0][2] = 60;
//
//    color_map[1][0] = 113;
//    color_map[1][1] = 179;
//    color_map[1][2] = 60;
//
//    color_map[2][0] = 185;
//    color_map[2][1] = 218;
//    color_map[2][2] = 255;
//
//    color_map[3][0] = 185;
//    color_map[3][1] = 218;
//    color_map[3][2] = 255;
//
//    color_map[4][0] = 0;
//    color_map[4][1] = 100;
//    color_map[4][2] = 0;
//
//    color_map[5][0] = 113;
//    color_map[5][1] = 179;
//    color_map[5][2] = 60;
//
//    color_map[6][0] = 128;
//    color_map[6][1] = 64;
//    color_map[6][2] = 0;
//
//    color_map[7][0] = 0;
//    color_map[7][1] = 255;
//    color_map[7][2] = 0;
//
//    color_map[8][0] = 255;
//    color_map[8][1] = 1;
//    color_map[8][2] = 0;
//
//    color_map[9][0] = 255;
//    color_map[9][1] = 192;
//    color_map[9][2] = 0;
//
//    color_map[10][0] = 255;
//    color_map[10][1] = 255;
//    color_map[10][2] = 0;
//
//    color_map[11][0] = 0;
//    color_map[11][1] = 178;
//    color_map[11][2] = 0;




//run
    if(map_){
//        color_map[0][0] = 113;
//        color_map[0][1] = 179;
//        color_map[0][2] = 60;
//
//        color_map[1][0] = 113;
//        color_map[1][1] = 179;
//        color_map[1][2] = 60;
//
//        color_map[2][0] = 185;
//        color_map[2][1] = 218;
//        color_map[2][2] = 255;
//
//        color_map[3][0] = 113;
//        color_map[3][1] = 179;
//        color_map[3][2] = 60;
//
//        color_map[4][0] = 0;
//        color_map[4][1] = 100;
//        color_map[4][2] = 0;
//
//        color_map[5][0] = 113;
//        color_map[5][1] = 179;
//        color_map[5][2] = 0;
    color_map[0][0] = 113;
    color_map[0][1] = 179;
    color_map[0][2] = 60;

    color_map[1][0] = 113;
    color_map[1][1] = 179;
    color_map[1][2] = 60;

    color_map[2][0] = 185;
    color_map[2][1] = 218;
    color_map[2][2] = 255;

    color_map[3][0] = 185;
    color_map[3][1] = 218;
    color_map[3][2] = 255;

    color_map[4][0] = 0;
    color_map[4][1] = 100;
    color_map[4][2] = 0;

    color_map[5][0] = 113;
    color_map[5][1] = 179;
    color_map[5][2] = 60;

    color_map[6][0] = 128;
    color_map[6][1] = 64;
    color_map[6][2] = 0;

    color_map[7][0] = 0;
    color_map[7][1] = 255;
    color_map[7][2] = 0;

    color_map[8][0] = 255;
    color_map[8][1] = 1;
    color_map[8][2] = 0;

    color_map[9][0] = 255;
    color_map[9][1] = 192;
    color_map[9][2] = 0;

    color_map[10][0] = 255;
    color_map[10][1] = 255;
    color_map[10][2] = 0;

    color_map[11][0] = 0;
    color_map[11][1] = 178;
    color_map[11][2] = 0;
    }else{
        color_map[0][0] = 255;
        color_map[0][1] = 191;
        color_map[0][2] = 0;

        color_map[1][0] = 255;
        color_map[1][1] = 245;
        color_map[1][2] = 152;

        color_map[2][0] = 0;
        color_map[2][1] = 255;
        color_map[2][2] = 255;

        color_map[3][0] = 0;
        color_map[3][1] = 255;
        color_map[3][2] = 255;

        color_map[4][0] = 0;
        color_map[4][1] = 100;
        color_map[4][2] = 0;

        color_map[5][0] = 0;
        color_map[5][1] = 0;
        color_map[5][2] = 254;

        color_map[6][0] = 128;
        color_map[6][1] = 64;
        color_map[6][2] = 128;

        color_map[7][0] = 205;
        color_map[7][1] = 0;
        color_map[7][2] = 0;

        color_map[8][0] = 255;
        color_map[8][1] = 1;
        color_map[8][2] = 1;

        color_map[9][0] = 255;
        color_map[9][1] = 192;
        color_map[9][2] = 0;

        color_map[10][0] = 255;
        color_map[10][1] = 255;
        color_map[10][2] = 127;

        color_map[11][0] = 0;
        color_map[11][1] = 178;
        color_map[11][2] = 237;
    }

    return color_map;
}

void LaneSegmentWorkerNode::CloudProject(const PointCloud2Adapter::MessageType &in_cloud_msg, cv::Mat& pred) {


    pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(in_cloud_msg, *in_cloud);
    std::unordered_map<cv::Point, pcl::PointXYZ> projection_map;


    std::vector<pcl::PointXYZ> cam_cloud(in_cloud->points.size());
    std::vector<cv::Point2f> projected_points_;
    std::vector<cv::Point3f> all_points_;

    for(size_t i = 0; i < in_cloud->points.size(); i++){
        all_points_.push_back(cv::Point3f(in_cloud->points[i].x, in_cloud->points[i].y, in_cloud->points[i].z));
    }


    if(in_cloud->points.size() > 0){
        cv::projectPoints(all_points_, rvec, tvec, camera_instrinsics_, distortion_coefficients_, projected_points_);
    }

    // points to image
    for (size_t i = 0; i < projected_points_.size(); i++)
    {
        //std::cout << "points size 1 : "<<projected_points_.size() << std::endl;
        cv::Point2f point = projected_points_[i];
        if(!(point.x >= 0 && point.x < image_width && point.y >=0 && point.y < image_height))
            continue;
        //  std::cout << "points size: "<<point.x << " "<< point.y << std::endl;
        // cv::circle(image_, point , 3, cv::Scalar(0,255,0), cv::FILLED);
        projection_map.insert(std::pair<cv::Point, pcl::PointXYZ>(cv::Point(point.x, point.y), in_cloud->points[i]));
    }



    //sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
    //postProcess(in_image_ptr->image, pred);


    //publisher_fused_image_.publish(*msg_);

//    out_cloud->points.clear();
//    out_cloud_->points.clear();


#pragma omp for
    //image to points

    if(map_){
        auto iter = projection_map.begin();

        while(iter != projection_map.end()){
            pcl::PointXYZRGB colored_3d_point;
            std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
            iterator_3d_2d = projection_map.find(cv::Point(iter->first.x, iter->first.y));
            cv::Vec3b rgb_pixel = pred.at<cv::Vec3b>(iter->first.y, iter->first.x);
            colored_3d_point.x = iterator_3d_2d->second.x;
            colored_3d_point.y = iterator_3d_2d->second.y;
            colored_3d_point.z = iterator_3d_2d->second.z;

            colored_3d_point.r = int(rgb_pixel[2]);
            colored_3d_point.g = int(rgb_pixel[1]);
            colored_3d_point.b = int(rgb_pixel[0]);
            if(rgb_pixel[2] != 0){
                out_cloud->points.push_back(colored_3d_point);
            }
            ++iter;
        }
    }else{
        auto iter = projection_map.begin();

        while(iter != projection_map.end()){
            pcl::PointXYZRGB colored_3d_point;
            std::unordered_map<cv::Point, pcl::PointXYZ>::const_iterator iterator_3d_2d;
            iterator_3d_2d = projection_map.find(cv::Point(iter->first.x, iter->first.y));
            cv::Vec3b rgb_pixel = pred.at<cv::Vec3b>(iter->first.y, iter->first.x);
            colored_3d_point.x = iterator_3d_2d->second.x;
            colored_3d_point.y = iterator_3d_2d->second.y;
            colored_3d_point.z = iterator_3d_2d->second.z+8;

            colored_3d_point.r = int(rgb_pixel[2]);
            colored_3d_point.g = int(rgb_pixel[1]);
            colored_3d_point.b = int(rgb_pixel[0]);
            if(rgb_pixel[2] == 152 || rgb_pixel[2] == 255){
                out_cloud->points.push_back(colored_3d_point);
            }
            ++iter;
        }

    }


    cv::Mat image_ = pred;
    for (size_t i = 0; i < projected_points_.size(); i++)
    {
        cv::Point2f point = projected_points_[i];
        if(!(point.x >= 0 && point.x < image_width && point.y >=0 && point.y < image_height))
            continue;
        cv::circle(image_, point , 3, cv::Scalar(0,255,0), cv::FILLED);
    }
    sensor_msgs::ImagePtr msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
   // publisher_fused_image_.publish(*msg_);
    MessageManager::PublishVisualSegmentProject(*msg_);



    // Publish PC
    sensor_msgs::PointCloud2 cloud_msg;

    pcl_ros::transformPointCloud(*out_cloud, *out_cloud, livox_vehicle_);

    pcl::toROSMsg(*out_cloud, cloud_msg);

    //pcl_ros::transformPointCloud(*out_cloud, *out_cloud, livox_vehicle_);

    cloud_msg.header = in_cloud_msg.header;
    cloud_msg.header.frame_id = "vehicle_link";
    cloud_msg.header.stamp = ros::Time::now();
    //publisher_fused_cloud_.publish(cloud_msg);

    MessageManager::PublishSemanticPointCloud2(cloud_msg);

}



void LaneSegmentWorkerNode::ShowObastacles(cv::Mat &cv_img, const CameraObstaclesAdapter::MessageType &out_objects)
{
    cv::Mat img = cv_img;
    for (size_t j = 0; j < out_objects.objects.size(); j++) {
        PerceptionObstacleAdapter::MessageType singleObject = out_objects.objects[j];
        int class_id = singleObject.type;
        cv::Rect r = cv::Rect(singleObject.x, singleObject.y, singleObject.width, singleObject.height);

        switch (class_id) {
            case 0:
                cv::rectangle(img, r, cv::Scalar(0, 255, 255), 3);
                break;
            case 1:
                cv::rectangle(img, r, cv::Scalar(132, 227, 255), 3);
                break;
            case 2:
                cv::rectangle(img, r, cv::Scalar(0, 0, 255), 3);
                break;
            case 3:
                cv::rectangle(img, r, cv::Scalar(0, 255, 0), 3);
                break;
            case 4:
                cv::rectangle(img, r, cv::Scalar(0, 0, 85), 3);
                break;
            case 5:
                cv::rectangle(img, r, cv::Scalar(42, 42, 128), 3);
                break;
            case 6:
                cv::rectangle(img, r, cv::Scalar(192, 192, 192), 3);
                break;
            case 7:
                cv::rectangle(img, r, cv::Scalar(100, 100, 192), 3);
                break;
            case 8:
                cv::rectangle(img, r, cv::Scalar(89, 32, 200), 3);
                break;
        }
        cv::putText(img,  singleObject.label + " id:" + std::to_string(singleObject.id), cv::Point(singleObject.x, singleObject.y - 1), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0, 0xFF), 2.5);
    }
}