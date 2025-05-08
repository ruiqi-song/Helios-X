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
#include "post_fusion.h"

PostFusion::PostFusion(){
    std::cout << "construct the PostFusion object succesfully!" << std::endl;
}
PostFusion::~PostFusion(){
    std::cout << "delete the PostFusion object succesfully" << std::endl;
}

void PostFusion::Init(tf::Transform &livox_vehicle, tf::Transform &camera_vehicle,
                      cv::Mat camera_instrinsics, cv::Mat distortion_coefficients){

    image_size_.width = 1920;
    image_size_.height = 1200;

    camera_vehicle_tf_.setData(camera_vehicle);
    vehicle_lidar_tf_.setData(livox_vehicle);

    camera_instrinsics_ = camera_instrinsics;
    distortion_coefficients_ = distortion_coefficients;

    fx_ = camera_instrinsics_.at<double>(0);
    fy_ = camera_instrinsics_.at<double>(4);
    cx_ = camera_instrinsics_.at<double>(2);
    cy_ = camera_instrinsics_.at<double>(5);

    std::string path = "/home/ricky/waytous_server/src/perception/conf/obstacles/surface/obstacles_attribution.yaml";

    read_service_config(path, obstacles_config_);

    for(YAML::const_iterator it = obstacles_config_["object"].begin(); it != obstacles_config_["object"].end(); ++it)
    {
        label_size_map.insert(std::make_pair(it->first.as<std::string>(), it->second["dimension"]["z"].as<double>()));
    }


//    read_extrinsics("camera_short_link", camera_livox_);
//    read_extrinsics("livox_lidar_link", livox_vehicle_);

//    tf::Transform camera_vehicle_;
//    camera_vehicle_ = livox_vehicle_ * camera_livox_.inverse();
//    camera_vehicle_ = camera_vehicle_.inverse();
//
//    camera_vehicle_tf_.setData(camera_vehicle_);
//    vehicle_lidar_tf_.setData(livox_vehicle_);


}

void PostFusion::Fusion(waytous_msgs::DetectedObjectArray &camera_obstacles,
                        waytous_msgs::DetectedObjectArray &lidar_obstacles,
                        waytous_msgs::DetectedObjectArray &radar_obstacles,
                        waytous_msgs::DetectedObjectArray &fusion_obstacles, cv::Mat &image_raw,
                        sensor_msgs::PointCloud2 &point_cloud, cv::Mat &fusion_image) {

    gLogWarning << "Helios perception system log info test" << std::endl;
    //gLogger.record(GSeverity::kWARNING, "Helios perception system log info test");
    fusion_obstacles.objects.clear();

    if(!tf_ok_){
        FindTransform();
    }

    PointCloudCallback(point_cloud);

    fusion_obstacles = SetState(camera_obstacles, lidar_obstacles, image_raw, fusion_image);

}

waytous_msgs::DetectedObjectArray PostFusion::SetState(waytous_msgs::DetectedObjectArray &camera_obstacles,
                                                       waytous_msgs::DetectedObjectArray &lidar_obstacles,
                                                       cv::Mat &image_raw, cv::Mat &fusion_image){

    waytous_msgs::DetectedObjectArray obstacles3D_in_camera;

    waytous_msgs::DetectedObjectArray obstacles3D_out_camera;
    waytous_msgs::DetectedObjectArray matched_obstacles3D;
    waytous_msgs::DetectedObjectArray unmatched_obstacles3D;
    waytous_msgs::DetectedObjectArray unmatched_obstacles2D;

    waytous_msgs::DetectedObjectArray fused_obstacles;

    fusion_image = image_raw.clone();

    TransformRangeToVision(lidar_obstacles, obstacles3D_in_camera, obstacles3D_out_camera); // split

    MatchedFromCameraToLidar(camera_obstacles, obstacles3D_in_camera);

    //matched obstacles
    for (int i = 0; i < obstacles3D_in_camera.objects.size(); ++i)
    {
        if (!obstacles3D_in_camera.objects.empty() && obstacles3D_in_camera.objects[i].fusion_state == 0)
        {
            fused_obstacles.objects.push_back(obstacles3D_in_camera.objects[i]);
        }
        if (!obstacles3D_in_camera.objects.empty() && obstacles3D_in_camera.objects[i].fusion_state != 0)
        {
            unmatched_obstacles3D.objects.push_back(obstacles3D_in_camera.objects[i]);
        }
    }

    for (int i = 0; i < camera_obstacles.objects.size(); ++i){
        if(camera_obstacles.objects[i].fusion_state != 0){
            unmatched_obstacles2D.objects.push_back(camera_obstacles.objects[i]);
        }

    }


    // unmatched camera obstacles
    for (size_t i = 0; i < camera_obstacles.objects.size(); i++)
    {
            auto object = camera_obstacles.objects[i];
            valid_points_mutex_.lock();
            InferObjectPose(object, fusion_image); // change object's position
            valid_points_mutex_.unlock();
            fused_obstacles.objects.push_back(object);
    }

//    for (size_t i = 0; i < unmatched_obstacles2D.objects.size(); i++)
//    {
//        auto object = unmatched_obstacles2D.objects[i];
//        valid_points_mutex_.lock();
//        InferObjectPose(object, fusion_image); // change object's position
//        valid_points_mutex_.unlock();
//        fused_obstacles.objects.push_back(object);
//    }

    //lidar obstacles out of camera view
    for (auto &object: obstacles3D_out_camera.objects)
    {
        fused_obstacles.objects.push_back(object);
    }

    fused_obstacles.header = lidar_obstacles.header;

    return fused_obstacles;

}

void PostFusion::MatchedFromCameraToLidar(waytous_msgs::DetectedObjectArray &camera_obstacles,
                                          waytous_msgs::DetectedObjectArray &obstacles3D_in_camera) {
    for (size_t i = 0; i < camera_obstacles.objects.size(); i++)
    {
        auto vision_object = camera_obstacles.objects[i];

        // skip matching for ROCK, SIGN, WARNING
        if(vision_object.label == "rock" ||
           vision_object.label == "sign" ||
           vision_object.label == "warning")
        {
            continue;
        }

        cv::Rect vision_rect(vision_object.x, vision_object.y,
                             vision_object.width, vision_object.height);
        int vision_rect_area = vision_rect.area();
        long closest_index = -1;
        double closest_distance = std::numeric_limits<double>::max();


        // iterate over range objects
        for (size_t j = 0; j < obstacles3D_in_camera.objects.size(); j++)
        {
            double current_distance = GetDistanceToObject(obstacles3D_in_camera.objects[j]); // object distance

            cv::Rect range_rect = ProjectDetectionToRect(obstacles3D_in_camera.objects[j]);

            int range_rect_area = range_rect.area();

            // if IoU > thres, assign attributes of vision detections to range detections
            cv::Rect overlap = range_rect & vision_rect;
            cv::Rect union_rect = range_rect | vision_rect;
            double iou = double(overlap.area()) / double(union_rect.area());


            if ((overlap.area() > range_rect_area * overlap_threshold_)
                || (overlap.area() > vision_rect_area * overlap_threshold_)
                    )
                // if (iou > overlap_threshold_)
            {
                if( image_ok_ &&
                    0 < range_rect.x < image_size_.width &&
                    0 < range_rect.br().x < image_size_.width &&
                    0 < range_rect.y < image_size_.height &&
                    0 < range_rect.br().y < image_size_.height) {
                    //cv::rectangle(image_proj,range_rect,cv::Scalar(255,0,0),1,1,0);
                    cv::circle(fusion_image_, cv::Point2f(vision_object.x + vision_object.width / 2,
                                                          vision_object.y + vision_object.height / 2), 5, CV_RGB(255, 0, 0), -1, 1, 0);
                    cv::rectangle(fusion_image_,range_rect,CV_RGB(255,0,0),1,1,0);
                }

                //matched_camera_lidar[i].push_back(j); // assign i to j
                obstacles3D_in_camera.objects[j].score = vision_object.score;
                obstacles3D_in_camera.objects[j].label = vision_object.label;
                obstacles3D_in_camera.objects[j].color = vision_object.color;
                obstacles3D_in_camera.objects[j].image_frame = vision_object.image_frame;
                obstacles3D_in_camera.objects[j].x = vision_object.x;
                obstacles3D_in_camera.objects[j].y = vision_object.y;
                obstacles3D_in_camera.objects[j].width = vision_object.width;
                obstacles3D_in_camera.objects[j].height = vision_object.height;
                obstacles3D_in_camera.objects[j].angle = vision_object.angle;
                obstacles3D_in_camera.objects[j].id = vision_object.id;
                CheckMinimumDimensions(obstacles3D_in_camera.objects[j]);
                if (vision_object.pose.orientation.x > 0
                    || vision_object.pose.orientation.y > 0
                    || vision_object.pose.orientation.z > 0)
                {
                    obstacles3D_in_camera.objects[j].pose.orientation = vision_object.pose.orientation;
                }
                if (current_distance < closest_distance)
                {
                    closest_index = j;
                    closest_distance = current_distance;
                }

                //matched_obstacles3D.objects.push_back(obstacles3D_in_camera.objects[j]);

                camera_obstacles.objects[i].fusion_state = 0;
                obstacles3D_in_camera.objects[j].fusion_state = 0;


                //matched_camera[i] = true; // true if i_th vision detection is used to assign
            }//end if overlap

        }//end for obstacles3D_in_camera
        // matched_lidar[i] = closest_index; // cloest range detection index for i_th vision detection
    }
}


void PostFusion::TransformRangeToVision(const waytous_msgs::DetectedObjectArray &in_range_detections,
        waytous_msgs::DetectedObjectArray &out_in_cv_range_detections,
        waytous_msgs::DetectedObjectArray &out_out_cv_range_detections)
{
    out_in_cv_range_detections.header = in_range_detections.header;
    out_in_cv_range_detections.objects.clear();
    out_out_cv_range_detections.header = in_range_detections.header;
    out_out_cv_range_detections.objects.clear();
    for (size_t i = 0; i < in_range_detections.objects.size(); i++)
    {
        if (IsObjectInImage(in_range_detections.objects[i]))
        {
            out_in_cv_range_detections.objects.push_back(in_range_detections.objects[i]);
        } else
        {
            out_out_cv_range_detections.objects.push_back(in_range_detections.objects[i]);
        }
    }
}

bool PostFusion::IsObjectInImage(const waytous_msgs::DetectedObject &in_detection)
{
    cv::Point3f image_space_point = TransformPoint(in_detection.pose.position, camera_vehicle_tf_);

    cv::Point2i image_pixel = ProjectPoint(image_space_point);

    return (image_pixel.x >= 0)
           && (image_pixel.x < image_size_.width)
           && (image_pixel.y >= 0)
           && (image_pixel.y < image_size_.height)
           && (image_space_point.z > 0);
}

double PostFusion::GetDistanceToObject(const waytous_msgs::DetectedObject &in_object)
{
    return sqrt(in_object.dimensions.x * in_object.dimensions.x +
                in_object.dimensions.y * in_object.dimensions.y +
                in_object.dimensions.z * in_object.dimensions.z);
}

cv::Rect PostFusion::ProjectDetectionToRect(const waytous_msgs::DetectedObject &in_detection)
{
    cv::Rect projected_box;

    Eigen::Vector3f pos;
    pos << in_detection.pose.position.x,
            in_detection.pose.position.y,
            in_detection.pose.position.z;

    Eigen::Quaternionf rot(in_detection.pose.orientation.w,
                           in_detection.pose.orientation.x,
                           in_detection.pose.orientation.y,
                           in_detection.pose.orientation.z);

    std::vector<double> dims = {
            in_detection.dimensions.x,
            in_detection.dimensions.y,
            in_detection.dimensions.z
    };

    jsk_recognition_utils::Cube cube(pos, rot, dims);

    Eigen::Affine3f range_vision_tf;
    tf::transformTFToEigen(camera_vehicle_tf_, range_vision_tf);
    jsk_recognition_utils::Vertices vertices = cube.transformVertices(range_vision_tf);

    std::vector<cv::Point> polygon;
    for (auto &vertex : vertices)
    {
        cv::Point p = ProjectPoint(cv::Point3f(vertex.x(), vertex.y(), vertex.z()));
        polygon.push_back(p);
    }

    projected_box = cv::boundingRect(polygon);

    return projected_box;
}

void PostFusion::CheckMinimumDimensions(waytous_msgs::DetectedObject &in_out_object)
{
    if(in_out_object.dimensions.x < obstacles_config_["object"][in_out_object.label]["dimension"]["x"].as<double>())
    {
        in_out_object.dimensions.x = obstacles_config_["object"][in_out_object.label]["dimension"]["x"].as<double>();
    }
    if(in_out_object.dimensions.y < obstacles_config_["object"][in_out_object.label]["dimension"]["y"].as<double>())
    {
        in_out_object.dimensions.y = obstacles_config_["object"][in_out_object.label]["dimension"]["y"].as<double>();
    }
    if(in_out_object.dimensions.z < obstacles_config_["object"][in_out_object.label]["dimension"]["z"].as<double>())
    {
        in_out_object.dimensions.z = obstacles_config_["object"][in_out_object.label]["dimension"]["z"].as<double>();
    }
}



void PostFusion::InferObjectPose(waytous_msgs::DetectedObject &object, cv::Mat &fusion_image){

    std::vector<cv::Point3f> origi_point_in;         // points in current object bbox
    std::vector<cv::Point3f> origi_point_neightor;   // points near current object bbox
    cv::Point2f left_top_p = cv::Point2f(object.x, object.y);// camera_object_list[i][0];
    cv::Point2f right_bottom_p = cv::Point2f(object.x+object.width, object.y+object.height); // camera_object_list[i][1];



    // iterate over projected points
    // to find in-bbox points and near-bbox points
    for (size_t j =0; j < projected_points_.size(); j++) {
        cv::Point2f p = projected_points_[j];
        if(isnan(p.x) or isnan(p.y))
            continue;
        if (!(p.x >= 0 && p.x < image_size_.width && p.y >= 0 && p.y < image_size_.height))
            continue;

        // find in-bbox points
        if(p.x > left_top_p.x && p.x < right_bottom_p.x && p.y > left_top_p.y && p.y < right_bottom_p.y) {
            cv::circle(fusion_image, p, 2, CV_RGB(0, 255, 0), -1, 1, 0); // green
            origi_point_in.push_back(all_points_[j]);
        } else {
            // find near-bbox points
            float aa = fabs(p.x - left_top_p.x);
            float bb = fabs(p.x - right_bottom_p.x);
            float cc = fabs(p.y - left_top_p.y);
            float dd = fabs(p.y - right_bottom_p.y);
            float ee = (aa > bb) ? bb : aa;
            float ff = (cc > dd) ? dd : cc;
            float gg = ee + ff;
            if(gg < 20) {
                cv::circle(fusion_image, p, 2, CV_RGB(0, 0, 255), -1, 1, 0); // blue
                origi_point_neightor.push_back(all_points_[j]);
            }
        }
    }
    float min_dis = 1000.0;
    int num_thres = 1;

    // if in-bbox points available
    if(origi_point_in.size() > num_thres) {
        // std::cout << "origi_point_in.size() is " << origi_point_in.size() << std::endl;
        float x_avg = 0;
        float y_avg = 0;
        float z_avg = 0;
        // float min_dis = 1000.0;
        for (size_t j =0; j < origi_point_in.size(); j++) {
            x_avg = x_avg + origi_point_in[j].x;
            y_avg = y_avg + origi_point_in[j].y;
            z_avg = z_avg + origi_point_in[j].z;
            float one_dis = sqrt(
                    origi_point_in[j].x * origi_point_in[j].x +
                    origi_point_in[j].y * origi_point_in[j].y +
                    origi_point_in[j].z * origi_point_in[j].z
            );
            if(one_dis < min_dis) min_dis = one_dis;
        }
        x_avg = x_avg / origi_point_in.size();
        y_avg = y_avg / origi_point_in.size();
        z_avg = z_avg / origi_point_in.size();
        // float obj_dis = sqrt(x_avg * x_avg + y_avg * y_avg + z_avg * z_avg );
        // std::cout << "matched obj_dis is " << obj_dis  << std::endl;
        object.pose.position.x = x_avg;
        object.pose.position.y = y_avg;
        object.pose.position.z = z_avg;
        object.fusion_state = 2;

    } else if(origi_point_neightor.size() > num_thres) {

        // if in-bbox points unavailable, but near-bbox points available, use near-bbox points
        // std::cout << "origi_point_neightor.size() is " << origi_point_neightor.size() << std::endl;
        float x_avg = 0;
        float y_avg = 0;
        float z_avg = 0;
        //
        for (size_t j =0; j < origi_point_neightor.size(); j++) {
            x_avg = x_avg + origi_point_neightor[j].x;
            y_avg = y_avg + origi_point_neightor[j].y;
            z_avg = z_avg + origi_point_neightor[j].z;
            float one_dis = sqrt(
                    origi_point_neightor[j].x * origi_point_neightor[j].x +
                    origi_point_neightor[j].y * origi_point_neightor[j].y +
                    origi_point_neightor[j].z * origi_point_neightor[j].z
            );
            if(one_dis < min_dis)min_dis = one_dis;
        }
        x_avg = x_avg / origi_point_neightor.size();
        y_avg = y_avg / origi_point_neightor.size();
        z_avg = z_avg / origi_point_neightor.size();
        // float obj_dis = sqrt(x_avg * x_avg + y_avg * y_avg + z_avg * z_avg );
        // std::cout << "neighbor obj_dis is " << obj_dis  << std::endl;
        object.pose.position.x = x_avg;
        object.pose.position.y = y_avg;
        object.pose.position.z = z_avg;
        object.fusion_state = 2;

    } else {
        // ranging_.KdTreePosition(object);

        cv::circle(fusion_image, cv::Point2f(object.x + object.width / 2, object.y + object.height / 2), 5, CV_RGB(255, 255, 255), -1, 1, 0); // white

        if (object.label != "sign" && object.label != "warning" && object.label != "sign"){
            ranging_.SizeRayPosition(object,
                                     fx_, fy_, cx_, cy_,
                                     image_size_,
                                     camera_vehicle_tf_.inverse(),
                                     label_size_map);


            object.fusion_state = 3;

        }

    }

    if (object.label != "sign" && object.label != "warning" && object.label != "rock"){
        cv::putText(fusion_image, std::to_string(int(object.pose.position.x - 24)) + "m", cv::Point(object.x, object.y + object.height + 20), cv::FONT_HERSHEY_COMPLEX, 1,
                    cv::Scalar(0, 0xFF, 0xFF), 2);
    }


}


void PostFusion::FindTransform()
{
    tf_ok_ = false;
    bool tf_o = false;
    try
    {
        tf_o = true;
        tvec_ = (cv::Mat_<float>(3,1) << camera_vehicle_tf_.getOrigin().getX(),
                camera_vehicle_tf_.getOrigin().getY(),camera_vehicle_tf_.getOrigin().getZ());

        double angle = camera_vehicle_tf_.getRotation().getAngle();
        tf::Vector3 axis = camera_vehicle_tf_.getRotation().getAxis().normalize();
        rvec_ = (cv::Mat_<float>(3,1) << angle*axis.getX(), angle*axis.getY(), angle*axis.getZ());
        tf_ok_ = true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("[%s] %s", "__APP_NAME__", ex.what());
    }
}


cv::Point3f PostFusion::TransformPoint(const geometry_msgs::Point &in_point, const tf::StampedTransform &in_transform)
{
    tf::Vector3 tf_point(in_point.x, in_point.y, in_point.z);
    tf::Vector3 tf_point_t = in_transform * tf_point;
    // ROS_INFO("lidar point: %f %f %f",in_point.x, in_point.y, in_point.z);
    // ROS_INFO("camera point: %f %f %f",tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
    return cv::Point3f(tf_point_t.x(), tf_point_t.y(), tf_point_t.z());
}

cv::Point2i PostFusion::ProjectPoint(const cv::Point3f &in_point)
{

    /* original implementation, igoring distortion */
    // auto u = int(in_point.x * fx_ / in_point.z + cx_);
    // auto v = int(in_point.y * fy_ / in_point.z + cy_);
    // return cv::Point2i(u, v);

    cv::Mat rvec = (cv::Mat_<float>(3,1) << 0,0,0);
    cv::Mat tvec = (cv::Mat_<float>(3,1) << 0,0,0);
    std::vector<cv::Point3f> in_points{in_point};
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(in_points, rvec, tvec, camera_instrinsics_, distortion_coefficients_, projected_points);

    return cv::Point2i(int(projected_points[0].x),int(projected_points[0].y));
}

void PostFusion::PointCloudCallback(const sensor_msgs::PointCloud2& in_pointcloud_msg){

    if (!tf_ok_)
    {
        FindTransform();
    }
    valid_points_mutex_.lock();

    all_points_.clear();
    projected_points_.clear();
    pcl::fromROSMsg(in_pointcloud_msg,point_cloud_);
    for (size_t i = 0; i < point_cloud_.points.size(); i++) {
        if(point_cloud_.points[i].x < 0)
            continue;
        geometry_msgs::Point p_lidar;
        p_lidar.x = point_cloud_.points[i].x;
        p_lidar.y = point_cloud_.points[i].y;
        p_lidar.z = point_cloud_.points[i].z;
        all_points_.push_back(TransformPoint(p_lidar, vehicle_lidar_tf_));
    }
    if(all_points_.size() > 0 && tf_ok_)
        cv::projectPoints(all_points_, rvec_, tvec_, camera_instrinsics_, distortion_coefficients_, projected_points_);

    valid_points_mutex_.unlock();

}


//waytous_msgs::DetectedObjectArray PostFusion::SetState(waytous_msgs::DetectedObjectArray &camera_obstacles,
//                                                       waytous_msgs::DetectedObjectArray &lidar_obstacles,
//                                                       cv::Mat &image_raw, cv::Mat &fusion_image){
//
//    waytous_msgs::DetectedObjectArray obstacles3D_in_camera;
//    waytous_msgs::DetectedObjectArray obstacles3D_out_camera;
//
//    waytous_msgs::DetectedObjectArray matched_obstacles3D;
//    waytous_msgs::DetectedObjectArray unmatched_obstacles3D;
//    waytous_msgs::DetectedObjectArray matched_obstacles2D;
//    waytous_msgs::DetectedObjectArray unmatched_obstacles2D;
//
//
//    waytous_msgs::DetectedObjectArray obstacles_in_camera;
//    waytous_msgs::DetectedObjectArray obstacles_out_camera;
//    waytous_msgs::DetectedObjectArray fused_obstacles;
//
//    TransformRangeToVision(lidar_obstacles, obstacles_in_camera, obstacles_out_camera); // split
//
//
//    fused_obstacles.header = lidar_obstacles.header;
//// overlap
//    std::vector<std::vector<size_t> > matched_camera_lidar(camera_obstacles.objects.size());
//
//    std::vector<bool> matched_camera(camera_obstacles.objects.size(), false);
//    std::vector<long> matched_lidar(camera_obstacles.objects.size());
//
//
//    cv::Mat image_proj = image_raw.clone();
//
//    fusion_image = image_raw.clone();
//
//    // iterate over camera objects
//    for (size_t i = 0; i < camera_obstacles.objects.size(); i++)
//    {
//
//        auto vision_object = camera_obstacles.objects[i];
//
//        // skip matching for ROCK, SIGN, WARNING
//        if(vision_object.label == "rock" ||
//           vision_object.label == "sign" ||
//           vision_object.label == "warning")
//        {
//            continue;
//        }
//
//        cv::Rect vision_rect(vision_object.x, vision_object.y,
//                             vision_object.width, vision_object.height);
//        int vision_rect_area = vision_rect.area();
//        long closest_index = -1;
//        double closest_distance = std::numeric_limits<double>::max();
//
//
//        // iterate over range objects
//        for (size_t j = 0; j < obstacles_in_camera.objects.size(); j++)
//        {
//            double current_distance = GetDistanceToObject(obstacles_in_camera.objects[j]); // object distance
//
//            cv::Rect range_rect = ProjectDetectionToRect(obstacles_in_camera.objects[j]);
//
//            int range_rect_area = range_rect.area();
//
//            // if IoU > thres, assign attributes of vision detections to range detections
//            cv::Rect overlap = range_rect & vision_rect;
//            cv::Rect union_rect = range_rect | vision_rect;
//            double iou = double(overlap.area()) / double(union_rect.area());
//
//            std::cout << "----------------------------------" <<std::endl;
//            std::cout << "iou: " << overlap.area() << " " << union_rect.area() << std::endl;
//
//
//            if ((overlap.area() > range_rect_area * overlap_threshold_)
//                || (overlap.area() > vision_rect_area * overlap_threshold_)
//                    )
//                // if (iou > overlap_threshold_)
//            {
//                if( image_ok_ &&
//                    0 < range_rect.x < image_size_.width &&
//                    0 < range_rect.br().x < image_size_.width &&
//                    0 < range_rect.y < image_size_.height &&
//                    0 < range_rect.br().y < image_size_.height) {
//                    cv::rectangle(image_proj,range_rect,cv::Scalar(255,0,0),1,1,0);
//                    cv::circle(fusion_image_, cv::Point2f(vision_object.x + vision_object.width / 2,
//                                                          vision_object.y + vision_object.height / 2), 5, CV_RGB(255, 0, 0), -1, 1, 0);
//                    cv::rectangle(fusion_image_,range_rect,CV_RGB(255,0,0),1,1,0);
//                }
//
//                matched_camera_lidar[i].push_back(j); // assign i to j
//                obstacles_in_camera.objects[j].score = vision_object.score;
//                obstacles_in_camera.objects[j].label = vision_object.label;
//                obstacles_in_camera.objects[j].color = vision_object.color;
//                obstacles_in_camera.objects[j].image_frame = vision_object.image_frame;
//                obstacles_in_camera.objects[j].x = vision_object.x;
//                obstacles_in_camera.objects[j].y = vision_object.y;
//                obstacles_in_camera.objects[j].width = vision_object.width;
//                obstacles_in_camera.objects[j].height = vision_object.height;
//                obstacles_in_camera.objects[j].angle = vision_object.angle;
//                obstacles_in_camera.objects[j].id = vision_object.id;
//                CheckMinimumDimensions(obstacles_in_camera.objects[j]);
//                if (vision_object.pose.orientation.x > 0
//                    || vision_object.pose.orientation.y > 0
//                    || vision_object.pose.orientation.z > 0)
//                {
//                    obstacles_in_camera.objects[i].pose.orientation = vision_object.pose.orientation;
//                }
//                if (current_distance < closest_distance)
//                {
//                    closest_index = j;
//                    closest_distance = current_distance;
//                }
//                matched_camera[i] = true; // true if i_th vision detection is used to assign
//            }//end if overlap
//        }//end for obstacles_in_camera
//        matched_lidar[i] = closest_index; // cloest range detection index for i_th vision detection
//    }
//
//
//    //std::vector<bool> used_range_detections(obstacles_in_camera.objects.size(), false);
//
//    //matched obstacles
//    for (size_t i = 0; i < matched_camera_lidar.size(); i++)
//    {
//        if (!obstacles_in_camera.objects.empty() && matched_lidar[i] >= 0)
//        {
//           // used_range_detections[i] = true;
//            // add matched range detections
//            // note: for i_th vision detection, choose closest range detection
//            fused_obstacles.objects.push_back(obstacles_in_camera.objects[matched_lidar[i]]);
//        }
//    }
//
//
//
//
//    // unmatched camera obstacles
//    for (size_t i = 0; i < matched_camera.size(); i++)
//    {
//        if (!matched_camera[i])
//        {
//            // 2D -> 3D (infer object's position according to 2d bbox)
//            auto object = camera_obstacles.objects[i];
//            valid_points_mutex_.lock();
//            InferObjectPose(object, fusion_image); // change object's position
//            valid_points_mutex_.unlock();
//            fused_obstacles.objects.push_back(object);
//        }
//    }
//
//
//    //lidar obstacles out of camera view
//    for (auto &object: obstacles_out_camera.objects)
//    {
//        fused_obstacles.objects.push_back(object);
//    }
//
//
//    //enable merged for visualization
//    for (auto &object : fused_obstacles.objects)
//    {
//        object.valid = true;
//    }
//
//
//    return fused_obstacles;
//
//}





