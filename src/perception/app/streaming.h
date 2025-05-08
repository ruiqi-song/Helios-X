//
// Created by 宋瑞琦 on 2021/7/2.
//

#pragma once
#include <memory>
#include <iostream>

#include "camera/detector/yolov5/yolov5.h"
#include "camera/segment/bisenet/bisenet.h"
#include "camera/tracker/deepsort/deepsorts.h"
#include "master/master_node.h"

class Streaming{
public:
    Streaming();
    ~Streaming();
    bool Init();
    bool Start();
    bool Stop();
    int Spin();
//private:
    //MasterNode perception_master_;
    //std::shared_ptr<CameraObstacleDetector> obstacleDetector_;
    //std::shared_ptr<CameraLaneDetector> laneDetector_;
    //std::shared_ptr<CameraObstacleTracker> obstacleTracker_;

};
/*
#define WAYTOUS_MAIN()                                      \
  int main(int argc, char **argv) {                         \
    Streaming streaming_;                                 \
    streaming_.Spin();                                     \
    return 0;                                               \
  }
  */