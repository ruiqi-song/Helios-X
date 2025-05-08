//
// Created by 宋瑞琦 on 2021/7/1.
//

#pragma once
namespace waytous {
    namespace perception {
        struct YOLO {
            struct YOLOV3 {
                int height = 512;
            } yolov3;
            struct YOLOV5 {
                int height = 608;
            } yolov5;
            int height = 120;
        } yolo;
    }
}