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

//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "common_shared_data.h"

struct LidarItem {
    //pcl::PointCloud<pcl::PointXYZI> data;
    sensor_msgs::PointCloud2 data;
    double timestamp = 0.0;
};

class LidarSharedData : public CommonSharedData<LidarItem>{
public:
    LidarSharedData() = default;
    virtual ~LidarSharedData() = default;

    std::string name() const override {
        return "LidarSharedData";
    }
   // int size() = default;

};
