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

// headers in STL
#include <cmath>
#include <iostream>

// headers in local files
#include "preprocess_anchor_based.h"

PreprocessPoints::PreprocessPoints(const int MAX_NUM_PILLARS, const int MAX_POINTS_PER_PILLAR, const int GRID_X_SIZE,
                                   const int GRID_Y_SIZE, const int GRID_Z_SIZE, const float PILLAR_X_SIZE,
                                   const float PILLAR_Y_SIZE, const float PILLAR_Z_SIZE, const float MIN_X_RANGE,
                                   const float MIN_Y_RANGE, const float MIN_Z_RANGE, const int NUM_INDS_FOR_SCAN,
                                   const int NUM_BOX_CORNERS)
  : MAX_NUM_PILLARS_(MAX_NUM_PILLARS)
  , MAX_NUM_POINTS_PER_PILLAR_(MAX_POINTS_PER_PILLAR)
  , GRID_X_SIZE_(GRID_X_SIZE)
  , GRID_Y_SIZE_(GRID_Y_SIZE)
  , GRID_Z_SIZE_(GRID_Z_SIZE)
  , PILLAR_X_SIZE_(PILLAR_X_SIZE)
  , PILLAR_Y_SIZE_(PILLAR_Y_SIZE)
  , PILLAR_Z_SIZE_(PILLAR_Z_SIZE)
  , MIN_X_RANGE_(MIN_X_RANGE)
  , MIN_Y_RANGE_(MIN_Y_RANGE)
  , MIN_Z_RANGE_(MIN_Z_RANGE)
  , NUM_INDS_FOR_SCAN_(NUM_INDS_FOR_SCAN)
  , NUM_BOX_CORNERS_(NUM_BOX_CORNERS)
{
}

void PreprocessPoints::initializeVariables(int* coor_to_pillaridx, float* sparse_pillar_map, float* pillar_x,
                                           float* pillar_y, float* pillar_z, float* pillar_i,
                                           float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped)
{
  for (int i = 0; i < GRID_Y_SIZE_; i++)
  {
    for (int j = 0; j < GRID_X_SIZE_; j++)
    {
      coor_to_pillaridx[i * GRID_X_SIZE_ + j] = -1;
    }
  }

  for (int i = 0; i < NUM_INDS_FOR_SCAN_; i++)
  {
    for (int j = 0; j < NUM_INDS_FOR_SCAN_; j++)
    {
      sparse_pillar_map[i * NUM_INDS_FOR_SCAN_ + j] = 0;
    }
  }

  for (int i = 0; i < MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_; i++)
  {
    pillar_x[i] = 0;
    pillar_y[i] = 0;
    pillar_z[i] = 0;
    pillar_i[i] = 0;
    x_coors_for_sub_shaped[i] = 0;
    y_coors_for_sub_shaped[i] = 0;
  }
}

void PreprocessPoints::preprocess(const float* in_points_array, int in_num_points, int* x_coors, int* y_coors,
                                  float* num_points_per_pillar, float* pillar_x, float* pillar_y, float* pillar_z,
                                  float* pillar_i, float* x_coors_for_sub_shaped, float* y_coors_for_sub_shaped,
                                  float* pillar_feature_mask, float* sparse_pillar_map, int* host_pillar_count)
{
  int pillar_count = 0;
  float x_coors_for_sub[MAX_NUM_PILLARS_] = { 0 };
  float y_coors_for_sub[MAX_NUM_PILLARS_] = { 0 };
  // init variables
  int coor_to_pillaridx[GRID_Y_SIZE_ * GRID_X_SIZE_];
  initializeVariables(coor_to_pillaridx, sparse_pillar_map, pillar_x, pillar_y, pillar_z, pillar_i,
                      x_coors_for_sub_shaped, y_coors_for_sub_shaped);

  //points to pillars
  for (int i = 0; i < in_num_points; i++)
  {
    int x_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 0] - MIN_X_RANGE_) / PILLAR_X_SIZE_);
    int y_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 1] - MIN_Y_RANGE_) / PILLAR_Y_SIZE_);
    int z_coor = std::floor((in_points_array[i * NUM_BOX_CORNERS_ + 2] - MIN_Z_RANGE_) / PILLAR_Z_SIZE_);
    if (x_coor < 0 || x_coor >= GRID_X_SIZE_ || y_coor < 0 || y_coor >= GRID_Y_SIZE_ || z_coor < 0 ||
        z_coor >= GRID_Z_SIZE_)
    {
      continue;
    }
    // reverse index
    int pillar_index = coor_to_pillaridx[y_coor * GRID_X_SIZE_ + x_coor];
    if (pillar_index == -1)
    {
      pillar_index = pillar_count;
      if (pillar_count >= MAX_NUM_PILLARS_)
      {
        break;
      }
      pillar_count += 1;
      coor_to_pillaridx[y_coor * GRID_X_SIZE_ + x_coor] = pillar_index;

      y_coors[pillar_index] = std::floor(y_coor);
      x_coors[pillar_index] = std::floor(x_coor);

      // float y_offset = PILLAR_Y_SIZE_/ 2 + MIN_Y_RANGE_;
      // float x_offset = PILLAR_X_SIZE_/ 2 + MIN_X_RANGE_;
      // TODO Need to be modified after proper trining code
      // Will be modified in ver 1.1
      y_coors_for_sub[pillar_index] = std::floor(y_coor) * PILLAR_Y_SIZE_ + -39.9f;
      x_coors_for_sub[pillar_index] = std::floor(x_coor) * PILLAR_X_SIZE_ + 0.1f;

      sparse_pillar_map[y_coor * NUM_INDS_FOR_SCAN_ + x_coor] = 1;
    }
    int num = num_points_per_pillar[pillar_index];
    if (num < MAX_NUM_POINTS_PER_PILLAR_)
    {
      pillar_x[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 0];
      pillar_y[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 1];
      pillar_z[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 2];
      pillar_i[pillar_index * MAX_NUM_POINTS_PER_PILLAR_ + num] = in_points_array[i * NUM_BOX_CORNERS_ + 3];
      num_points_per_pillar[pillar_index] += 1;
    }
  }

  //
  for (int i = 0; i < MAX_NUM_PILLARS_; i++)
  {
    float x = x_coors_for_sub[i];
    float y = y_coors_for_sub[i];
    int num_points_for_a_pillar = num_points_per_pillar[i];
    for (int j = 0; j < MAX_NUM_POINTS_PER_PILLAR_; j++)
    {
      x_coors_for_sub_shaped[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = x;
      y_coors_for_sub_shaped[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = y;
      if (j < num_points_for_a_pillar)
      {
        pillar_feature_mask[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = 1.0f;
      }
      else
      {
        pillar_feature_mask[i * MAX_NUM_POINTS_PER_PILLAR_ + j] = 0.0f;
      }
    }
  }
  host_pillar_count[0] = pillar_count;
}


//void PreprocessPoints::preprocess(float *points, float *feature, int *indices, int pointNum, int pointDim) {
//    int size_ =  BEV_W * BEV_H;
//    for(int idx=0; idx< static_cast<int>(MAX_NUM_PILLARS_); idx++){
//        indices[idx] = -1;
//    }
//
//
//    for(int idx=0; idx<static_cast<int>( MAX_NUM_PILLARS_)*static_cast<int>(FEATURE_NUM)*static_cast<int>(MAX_POINTS_PER_PILLAR); idx++){
//        feature[idx] = 0;
//    }
//
//
//
//
//    std::vector<std::thread> threadPool;
//    for(int idx=0; idx <static_cast<int>(  THREAD_NUM); idx++){
//        std::thread worker(PreprocessWorker,
//                           points,
//                           feature,
//                           indices,
//                           pointNum,
//                           idx,
//                           MAX_NUM_PILLARS_/THREAD_NUM,
//                           pointDim
//        );
//
//        threadPool.push_back(std::move(worker));
//    }
//
//    for(auto idx=0; idx < THREAD_NUM; idx++){
//        threadPool[idx].join();
//    }
//
//
//}
//
//void PreprocessPoints::preprocessGPU(float *points, float *feature, int *indices, bool *_PMask, int *_PBEVIdxs,
//                                     int *_PPointNumAssigned, int *_BEVVoxelIdx, float *_VPointSum, int *_VRange,
//                                     int *_VPointNum, int pointNum, int pointDim) {
//    pointNum = pointNum > MAX_POINTS ? MAX_POINTS : pointNum;
//
//    //  GPU_CHECK(cudaMemset(feature, 0.0, MAX_PILLARS * MAX_PIONT_IN_PILLARS * FEATURE_NUM * sizeof(float)));
//    cudaMemset(feature, 0.0, MAX_NUM_PILLARS_ * MAX_NUM_POINTS_PER_PILLAR_ * FEATURE_NUM * sizeof(float));
//
//    _preprocess_gpu( dev_points, feature, indices,
//                     _PMask, _PBEVIdxs,  _PPointNumAssigned,  _BEVVoxelIdx, _VPointSum,  _VRange,  _VPointNum,
//                     pointNum);
//
//}
