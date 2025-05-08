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

#include "datatype.h"

#include <map>

//A tool to calculate distance;
class NearNeighborDisMetric{
public:
    enum METRIC_TYPE{euclidean=1, cosine};
    NearNeighborDisMetric(METRIC_TYPE metric,
            float matching_threshold,
            int budget);
    DYNAMICM distance(const FEATURESS& features, const std::vector<int> &targets);
    //    void partial_fit(FEATURESS& features, std::vector<int> targets, std::vector<int> active_targets);
    void partial_fit(std::vector<TRACKER_DATA>& tid_feats, std::vector<int>& active_targets);
    float mating_threshold;

private:
    typedef Eigen::VectorXf (NearNeighborDisMetric::*PTRFUN)(const FEATURESS&, const FEATURESS&);
    Eigen::VectorXf _nncosine_distance(const FEATURESS& x, const FEATURESS& y);
    Eigen::VectorXf _nneuclidean_distance(const FEATURESS& x, const FEATURESS& y);

    Eigen::MatrixXf _pdist(const FEATURESS& x, const FEATURESS& y);
    Eigen::MatrixXf _cosine_distance(const FEATURESS & a, const FEATURESS& b, bool data_is_normalized = false);
private:
    PTRFUN _metric;
    int budget;
    std::map<int, FEATURESS > samples;
};


