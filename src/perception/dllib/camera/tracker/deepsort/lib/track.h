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

#include "kalmanfilter.h"
#include "datatype.h"
#include "model.hpp"

class Track
{
    /*"""
    A single target track with state space `(x, y, a, h)` and associated
    velocities, where `(x, y)` is the center of the bounding box, `a` is the
    aspect ratio and `h` is the height.

    Parameters
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    n_init : int
        Number of consecutive detections before the track is confirmed. The
        track state is set to `Deleted` if a miss occurs within the first
        `n_init` frames.
    max_age : int
        The maximum number of consecutive misses before the track state is
        set to `Deleted`.
    feature : Optional[ndarray]
        Feature vector of the detection this track originates from. If not None,
        this feature is added to the `features` cache.

    Attributes
    ----------
    mean : ndarray
        Mean vector of the initial state distribution.
    covariance : ndarray
        Covariance matrix of the initial state distribution.
    track_id : int
        A unique track identifier.
    hits : int
        Total number of measurement updates.
    age : int
        Total number of frames since first occurance.
    time_since_update : int
        Total number of frames since last measurement update.
    state : TrackState
        The current track state.
    features : List[ndarray]
        A cache of features. On each measurement update, the associated feature
        vector is added to this list.

    """*/
    enum TrackState {Tentative = 1, Confirmed, Deleted};

public:
    Track(KAL_MEAN& mean, KAL_COVA& covariance, int track_id,
          int n_init, int max_age, const FEATURE& feature);
    Track(KAL_MEAN& mean, KAL_COVA& covariance, int track_id,
          int n_init, int max_age, const FEATURE& feature, int cls, float conf);
    void predit(KalmanFilter* kf);
    void update(KalmanFilter* const kf, const DETECTION_ROW &detection);
    void update(KalmanFilter* const kf, const DETECTION_ROW & detection, CLSCONF pair_det);
    void mark_missed();
    bool is_confirmed();
    bool is_deleted();
    bool is_tentative();
    DETECTBOX to_tlwh();
    int time_since_update;
    int track_id;
    FEATURESS features;
    KAL_MEAN mean;
    KAL_COVA covariance;

    int hits;
    int age;
    int _n_init;
    int _max_age;
    TrackState state;

    int cls;
    float conf;
private:
    void featuresAppendOne(const FEATURE& f);
};


