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

#pragma once

#include <iostream>
#include <functional>
#include <map>

#include <ros/ros.h>

#include "message.h"
#include "designer/singleton.h"
#include "topic.h"
#include "wrapper.h"

#define REGISTER_ADAPTER(name)                                                              \
public:                                                                                     \
    static void Init##name(const std::string &adapter_name,                                 \
            const std::string &topic_name, const std::string &flag) {                       \
        instance()->CreateSubPub##name(adapter_name, topic_name, flag);                     \
    }                                                                                       \
    static void Add##name##Callback(name##Adapter::Callback callback){                      \
        instance()->name##_->AddCallback(callback);                                         \
    }                                                                                       \
    template <class T>                                                                      \
    static void Add##name##Callback(                                                        \
          void (T::*fp)(const name##Adapter::MessageType &data), T *obj) {                  \
        Add##name##Callback(std::bind(fp, obj, std::placeholders::_1));                     \
    }                                                                                       \
    template <class T>                                                                      \
    static void Add##name##Callback(                                                        \
          void (T::*fp)(const name##Adapter::MessageType &data)) {                          \
        Add##name##Callback(fp);                                                            \
    }                                                                                       \
    static void Publish##name(const name##Adapter::MessageType &data) {                     \
        instance()->InternalPublish##name(data);                                            \
    }                                                                                       \
private:                                                                                    \
    std::unique_ptr<name##Adapter> name##_;                                                 \
    ros::Publisher name##publisher_;                                                        \
    ros::Subscriber name##subscriber_;                                                      \
                                                                                            \
    void CreateSubPub##name(const std::string &adapter_name,                                \
            const std::string &topic_name, const std::string &flag) {                       \
        name##_.reset(                                                                      \
                new name##Adapter(#name, topic_name));                                      \
                if(flag == RECEIVE_ONLY || flag == DUPLEX){                                 \
                    name##subscriber_ = node_handle_->subscribe(topic_name, 1,              \
                            &name##Adapter::StartCallbacks, name##_.get());                 \
                }                                                                           \
                if(flag == PUBLISH_ONLY || flag == DUPLEX){                                 \
                    name##publisher_ = node_handle_->advertise<name##Adapter::MessageType>  \
                            (topic_name, 1);                                                \
                }                                                                           \
    }                                                                                       \
    void InternalPublish##name(const name##Adapter::MessageType &data) {                    \
        if (!name##publisher_.getTopic().empty()) {                                         \
            name##publisher_.publish(data);                                                 \
        }                                                                                   \
    }

//template<typename T>
//typedef void (*func)(T callback,void *p1);//定义一个函数指针

using callback = std::function<void()>;

using callbacks_t = std::map<std::type_index, std::unique_ptr<Func_t>>;


//using FrontLongCamera = Cb_t<std::string, void()>


class MessageManager{
public:
    static void Init();

    using FrontLongCamera = Cb_t<std::string, int>;

    callbacks_t callbacks;

//    template <typename T>
//    struct Wrapper
//    {
//        typedef int (* CallbackFun) (const T& callback);
//    };





    static void FrontCameraCall(const int& w);
    static void RearCameraCall(const float& w);
    static void LeftCameraCall(const std::string& w);


    static int front;
    static float rear;
    static std::string left;

    static callback FrontCameraCall_;
    static callback RearCameraCall_;
    static callback LeftCameraCall_;

//
//
//
    typedef std::map<std::string, callback> new_map;

    static new_map callback_map;

//
    static  new_map BuildMap();

//

//
    static void GetFuncByName(std::string& name);


private:

    std::unique_ptr<ros::NodeHandle> node_handle_;

    REGISTER_ADAPTER(ImageRaw);
    REGISTER_ADAPTER(FrontLongCamera);
    REGISTER_ADAPTER(FrontShortCamera);
    REGISTER_ADAPTER(FrontLeftCamera);
    REGISTER_ADAPTER(FrontRightCamera);
    REGISTER_ADAPTER(LeftLateralCamera);
    REGISTER_ADAPTER(RightLateralCamera);
    REGISTER_ADAPTER(RearCamera);

    REGISTER_ADAPTER(PointCloud2);
    REGISTER_ADAPTER(Ouster64);
    REGISTER_ADAPTER(Horizon);
    REGISTER_ADAPTER(RSLidar16);

    REGISTER_ADAPTER(FrontRadar);
    REGISTER_ADAPTER(LeftRadar);
    REGISTER_ADAPTER(RightRadar);
    REGISTER_ADAPTER(RearRadar);

    REGISTER_ADAPTER(GpsOdom);
    REGISTER_ADAPTER(SemanticPointCloud2);
    REGISTER_ADAPTER(SemanticImageRaw);

    REGISTER_ADAPTER(CameraObstacles);
    REGISTER_ADAPTER(LidarObstacles);
    REGISTER_ADAPTER(RadarObstacles);
    REGISTER_ADAPTER(FusionObstacles);
    REGISTER_ADAPTER(LaneObstacles);
    REGISTER_ADAPTER(PredictTrajectory);

    REGISTER_ADAPTER(VisualCameraObstacles);
    REGISTER_ADAPTER(VisualLidarObstacles);
    REGISTER_ADAPTER(VisualFusionObstacles);
    REGISTER_ADAPTER(VisualHistoryPoints);
    REGISTER_ADAPTER(VisualHistoryTrajectory);
    REGISTER_ADAPTER(VisualSelf);
    REGISTER_ADAPTER(VisualPredictTrajectory);
    REGISTER_ADAPTER(VisualSegmentProject);
    REGISTER_ADAPTER(VisualSegmentContour);
    REGISTER_ADAPTER(VisualFusionImage);
    REGISTER_ADAPTER(VisualCameraImage);



    DECLARE_SINGLETON(MessageManager);




};