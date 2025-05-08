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

#include "trajectory_predict_node.h"


bool TrajectoryPredictNode::InitConfig(YAML::Node& config) {

}

bool TrajectoryPredictNode::InitMessage() {
    sub_fusion_obstacles_ = nh_.subscribe<waytous_msgs::DetectedObjectArray>("/fusion/obstacles", 1, &TrajectoryPredictNode::TrajectoryCallback, this);

    trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/fusion/trajectory_marker", 10 );
    //points_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/fusion/points_marker", 10 );
    //predict_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>( "/fusion/predict_marker", 10 );
}

bool TrajectoryPredictNode::InitModule() {

}

bool TrajectoryPredictNode::InitWorkerNode(YAML::Node& config) {
    InitMessage();
    readTxt("/home/ricky/windy.txt");
}

void TrajectoryPredictNode::RegistAllAlgorithms() {

}

bool TrajectoryPredictNode::InitSharedData() {

}

void TrajectoryPredictNode::TrajectoryCallback(const waytous_msgs::DetectedObjectArray::ConstPtr &fusion_obstacles) {

//    ros::Time time = fusion_obstacles->header.stamp;
//    GetHistoryTrajectory(time);
     //auto cur_gps = gps_shared_data_->getMsgNearestTime(ros::Time::now());
     auto cur_gps = gps_shared_data_->getMsgNowTime();
     GPSPoints point;
     point.x = cur_gps->x;
     point.y = cur_gps->y;
     point.z = cur_gps->z;
     Planning(point);


}


void TrajectoryPredictNode::PublishTrajectory() {

}

void TrajectoryPredictNode::GetHistoryTrajectory(ros::Time &stamp) {

    std::vector<std::vector<waytous_msgs::DetectedObject>> trace_;

    std::vector<int> vec_id;

    visualization_msgs::MarkerArray line_array_;
    visualization_msgs::MarkerArray points_array_;
    visualization_msgs::MarkerArray arrays_array_;


    auto this_objects = fusion_shared_obstacle_->getMsgNearestTime(stamp);


    for(int i = 0; i < this_objects->data.objects.size(); ++i){
        if(this_objects->data.objects[i].id != 0){
            vec_id.push_back(this_objects->data.objects[i].id);
        }
    }


    for(int k = 0; k < vec_id.size(); ++k){
        std::vector<waytous_msgs::DetectedObject> vec_;
        for(int i = 0; i < fusion_shared_obstacle_->size(); i++){
            waytous_msgs::DetectedObjectArray obstacleArray_ = fusion_shared_obstacle_->getMsgWithId(i)->data;
            for(int j = 0; j < obstacleArray_.objects.size(); j++){
                if(vec_id[k] == obstacleArray_.objects[j].id){
                    vec_.push_back(obstacleArray_.objects[j]);
                }

            }
        }
        trace_.push_back(vec_);
    }


    for(int i = 0; i < trace_.size(); ++i){

        visualization_msgs::Marker line_;
        visualization_msgs::Marker points_;
        visualization_msgs::Marker array_;
        if(trace_[i].size() > 1){
            //
            //if((abs(trace_[i].data()->pose.position.x) + abs(trace_[i].data()->pose.position.y)) < 100){
            if((abs(trace_[i].data()->pose.position.x) + abs(trace_[i].data()->pose.position.y)) < 100){
                ShowHistoryTrajectory(trace_[i], line_, points_, array_);
                line_array_.markers.push_back(line_);
                line_array_.markers.push_back(points_);
                line_array_.markers.push_back(array_);
//                points_array_.markers.push_back(points_);
//                arrays_array_.markers.push_back(array_);
            }

        }

    }


    trajectory_publisher_.publish(line_array_);
    //points_publisher_.publish(points_array_);
    //predict_publisher_.publish(arrays_array_);



}

bool TrajectoryPredictNode::ShowHistoryTrajectory(std::vector<waytous_msgs::DetectedObject> &vec, visualization_msgs::Marker &line,
                           visualization_msgs::Marker &points, visualization_msgs::Marker &array) {

    int size = vec.size();

    if(size < 3) return false;

    for(int i = 1; i < size - 1; i++){

        // visualization_msgs::Marker line_, points_;
        line.header.frame_id = "vehicle_link";
        points.header.frame_id = "vehicle_link";
        line.header.stamp = ros::Time::now();
        points.header.stamp = ros::Time::now();
        line.ns = points.ns = "points_and_lines";
        line.action = points.action = visualization_msgs::Marker::ADD;


        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 0.5;
        line.color.g = 1;
        line.color.a = 1;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.r = 1.0f;
        points.color.a = 1.0;

        geometry_msgs::Point p, q;
        p.x = vec[i].pose.position.x;
        p.y = vec[i].pose.position.y;
        p.z = vec[i].pose.position.z;

        q.x = vec[i + 1].pose.position.x;
        q.y = vec[i + 1].pose.position.y;
        q.z = vec[i + 1].pose.position.z;

        line.points.push_back(p);
        line.points.push_back(q);
        points.points.push_back(p);
        points.points.push_back(q);

        line.lifetime = ros::Duration(0.2);
        points.lifetime = ros::Duration(0.2);


    }

    array.header.frame_id = "vehicle_link";
    array.header.stamp = ros::Time::now();
    array.ns = "array_shapes";
    array.id = 1;

    array.type = visualization_msgs::Marker::ARROW;
    array.action = visualization_msgs::Marker::ADD;
    array.pose.position.z = vec[size - 1].pose.position.z;
    array.pose.position.y = vec[size - 1].pose.position.y;

    tf::Quaternion orit;


    if((vec[size - 1].pose.position.x - vec[size - 2].pose.position.x) > 0){
        double alpha  = atan((vec[size - 1].pose.position.y - vec[size - 2].pose.position.y) /
                                     (vec[size - 1].pose.position.x - vec[size - 2].pose.position.x));
        orit = tf::createQuaternionFromRPY(0, 0 ,alpha);
        array.pose.orientation.x = orit.getX();
        array.pose.orientation.y = orit.getY();
        array.pose.orientation.z = orit.getZ();
        array.pose.orientation.w = orit.getW();
        array.pose.position.x = vec[size - 1].pose.position.x + vec[size - 1].dimensions.x / 2;
    } else{
        double alpha  = atan((vec[size - 1].pose.position.y - vec[size - 2].pose.position.y) /
                                     (vec[size - 1].pose.position.x - vec[size - 2].pose.position.x));
        orit = tf::createQuaternionFromRPY(0, 0 ,3.14 + alpha);
        array.pose.orientation.x = orit.getX();
        array.pose.orientation.y = orit.getY();
        array.pose.orientation.z = orit.getZ();
        array.pose.orientation.w = orit.getW();
        array.pose.position.x = vec[size - 1].pose.position.x - vec[size - 1].dimensions.x / 2;

    }




    array.scale.x = 20;
    array.scale.y = 0.4;

    array.scale.z = 0.4;
    array.color.r = 1;
    array.color.g = 0;
    array.color.b = 0;
    array.color.a = 1;

    array.lifetime = ros::Duration(0.2);

    return true;

}

void TrajectoryPredictNode::readTxt(std::string file)
{
    std::ifstream infile;
    infile.open(file.data());
    assert(infile.is_open());

    std::vector<std::string> t;
//    std::vector<GPSPoints> gps_points;
    GPSPoints gps_point;
    std::string s;
    std::string s1;



    while(getline(infile,s))
    {
        std::stringstream input(s);
        input>>s1;
        gps_point.x = std::stof(s1);
        input>>s1;
        gps_point.y = std::stof(s1);
        input>>s1;
        gps_point.z = std::stof(s1);
        gps_points_.push_back(gps_point);


//        for(int i = 0; i< 2; ++i){
//            input>>s1
//        }
//        while(input>>s1)
//        {
//            t.push_back(s1);
//            gps_point.x
//        }
    }
    infile.close();

//    for(int i = 0; i < t.size() - 1; ++i){
//        gLogInfo << t[i] << " " << t[i + 1] << std::endl;
//        i += 1;
//    }
//         for(int i = 0; i < gps_points.size() - 1; ++i){
//             gLogInfo << gps_points[i].x << " " << gps_points[i].y << std::endl;
//         }


}

bool TrajectoryPredictNode::ShowTrajectory(std::vector<GPSPoints> &vec, visualization_msgs::Marker &line,
                                          visualization_msgs::Marker &points, int &location, float &speed) {
    int state = 0;
    float l = std::pow(std::pow(vec[location+30].x - vec[location].x, 2) + std::pow(vec[location+30].y - vec[location].y, 2),0.5);
    if(l > 1){
        state  = 1;
    }

    speed = l * 3.6;
    if(speed < 1){
        speed = 0;
    }
    if(speed > 35){
        speed = 35 + rand()/double(RAND_MAX);
    }

    gLogInfo << " the trunk speed is : " << speed << std::endl;

    for (int i = location+100; i < location + 600 ; i++) {
        // visualization_msgs::Marker line_, points_;
        line.header.frame_id = "world_link";
        points.header.frame_id = "world_link";
        line.header.stamp = ros::Time::now();
        points.header.stamp = ros::Time::now();
        line.ns = points.ns = "points_and_lins";
        line.action = points.action = visualization_msgs::Marker::ADD;


        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.scale.x = 4;
        if(state == 0){
            line.color.r = 1;
        } else{
            line.color.g = 1;
        }

        line.color.a = 1;

        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        points.color.g = 1.0f;
        points.color.a = 1.0;

        geometry_msgs::Point p, q;

        if(i < gps_points_.size()){
            p.x = vec[i].x;
            p.y = vec[i].y;
            p.z = vec[i].z + 6;


            q.x = vec[i + 1].x;
            q.y = vec[i + 1].y;
            q.z = vec[i + 1].z + 6;


            line.points.push_back(p);
            line.points.push_back(q);
            points.points.push_back(p);
            points.points.push_back(q);

            line.lifetime = ros::Duration(0.8);
            points.lifetime = ros::Duration(0.8);
        }

    }
}

bool TrajectoryPredictNode::GetTrajectory(std::vector<GPSPoints> &gps_points, std::vector<GPSPoints> &planning_points){
    planning_points.clear();

    int nums = gps_points.size();
    int num = nums - 1;

    for(int i = 0; i < 1000; ++i){
        int t = i * 0.001;
        float x,y = 0;
        for(int j = 0; j < nums; ++j){
            float b = std::pow(factorial(num) * t, j) * std::pow((1 - t), num - j)/
                      (factorial(j) * factorial(num - j));
            x += gps_points[j].x * b;
            y += gps_points[j].y * b;
        }
        GPSPoints tem_point;
        tem_point.x = x;
        tem_point.y = y;
        planning_points.push_back(tem_point);
    }

}

bool TrajectoryPredictNode::Planning(GPSPoints & position) {

    visualization_msgs::Marker line_;
    visualization_msgs::Marker points_;
    visualization_msgs::Marker speed_;
    visualization_msgs::Marker self_;

    float speed = 0;

    int size_ = gps_points_.size();

    int location = 0;

    for(int i = 0; i < size_ - 1; ++i){
        if(abs(gps_points_[i].x - position.x) <= 1e-2 &&
                abs(gps_points_[i].y - position.y) <= 1e-2){
            break;
        }
        else{
            location += 1;
        }
    }

    ShowTrajectory(gps_points_, line_, points_, location, speed);
    ShowSelfWithSpeed(speed_, self_, speed);

    visualization_msgs::MarkerArray marker_points_;
    visualization_msgs::MarkerArray marker_line_;
    visualization_msgs::MarkerArray marker_self_;


    int size = gps_points_.size();
//    for(int i = 0; i < 1000 - 1; ++i){
    marker_points_.markers.push_back(points_);
    marker_line_.markers.push_back(line_);
    marker_self_.markers.push_back(speed_);
    //marker_self_.markers.push_back(self_);
//    }


    MessageManager::PublishVisualHistoryPoints(marker_points_);
    MessageManager::PublishVisualHistoryTrajectory(marker_line_);
    MessageManager::PublishVisualPredictTrajectory(marker_self_);

    sleep(1);




}


bool TrajectoryPredictNode::ShowSelfWithSpeed(VisualMarkerAdapter::MessageType &text, VisualMarkerAdapter::MessageType &self, float &speed) {

    text.header.frame_id = "vehicle_link";
    text.header.stamp = ros::Time::now();
    text.ns = "basic_shapes";
    text.id = 1;

    text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::Marker::ADD;

    text.pose.orientation.x = 0;
    text.pose.orientation.y = 0;
    text.pose.orientation.z = 0;
    text.pose.orientation.w = 1;
    text.text = "speed: " + std::to_string(speed);

    text.scale.x = 6;
    text.scale.y = 6;

    text.scale.z = 6;
    text.color.r = 1;
    text.color.g = 0;
    text.color.b = 0;
    text.color.a = 1;

    text.pose.position.x = 13;
    text.pose.position.y = 0;
    text.pose.position.z = 7.5+8;

    text.lifetime = ros::Duration(0.2);


//    self.header.frame_id = "vehicle_link";
//    self.header.stamp = ros::Time::now();
//    self.ns = "box";
//    self.id = 1;
//
//    self.type = visualization_msgs::Marker::CUBE;
//    self.action = visualization_msgs::Marker::ADD;
//
//    self.pose.orientation.x = 0;
//    self.pose.orientation.y = 0;
//    self.pose.orientation.z = 0;
//    self.pose.orientation.w = 1;
//
//
//    self.scale.x = 12.94;
//    self.scale.y = 7.97;
//
//    self.scale.z = 7.12;
//    self.color.r = 1;
//    self.color.g = 1;
//    self.color.b = 0;
//    self.color.a = 1;
//
//    self.pose.position.x = 6.5;
//    self.pose.position.y = 0;
//    self.pose.position.y = 0;







}