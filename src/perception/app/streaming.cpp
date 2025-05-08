//
// Created by 宋瑞琦 on 2021/7/2.
//



#include "streaming.h"

Streaming::Streaming(){
    //laneDetector_.reset(new CameraLaneDetector());
    //obstacleTracker_.reset(new CameraObstacleTracker());
    //obstacleDetector_.reset(new CameraObstacleDetector());
    std::cout << "build perception" << std::endl;
}
Streaming::~Streaming(){
    std::cout << "delete perception" << std::endl;
}


bool Streaming::Init() {
    std::cout << "init perception config" << std::endl;
    return true;


}

bool Streaming::Start() {
    std::cout << "start perception" << std::endl;
    return true;

}

bool Streaming::Stop() {
    std::cout << "stop perception" << std::endl;
    return true;

}

int Streaming::Spin() {
    std::cout << "spin perception" << std::endl;
    return 1;

}


