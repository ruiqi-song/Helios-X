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

/**
* @file scatter_cuda.h
* @brief CUDA code for scatter operation
* @author Kosuke Murakami
* @date 2019/02/26
*/

#pragma once

class ScatterCuda
{
private:
  const int NUM_THREADS_;
  const int MAX_NUM_PILLARS_;
  const int GRID_X_SIZE_;
  const int GRID_Y_SIZE_;

public:
  /**
  * @brief Constructor
  * @param[in] NUM_THREADS The number of threads to launch cuda kernel
  * @param[in] MAX_NUM_PILLARS Maximum number of pillars
  * @param[in] GRID_X_SIZE Number of pillars in x-coordinate
  * @param[in] GRID_Y_SIZE Number of pillars in y-coordinate
  * @details Captital variables never change after the compile
  */
  ScatterCuda(const int NUM_THREADS, const int MAX_NUM_PILLARS, const int GRID_X_SIZE, const int GRID_Y_SIZE);

  /**
  * @brief Call scatter cuda kernel
  * @param[in] pillar_count The valid number of pillars
  * @param[in] x_coors X-coordinate indexes for corresponding pillars
  * @param[in] y_coors Y-coordinate indexes for corresponding pillars
  * @param[in] pfe_output Output from Pillar Feature Extractor
  * @param[out] scattered_feature Gridmap representation for pillars' feature
  * @details Allocate pillars in gridmap based on index(coordinates) information
  */
  void doScatterCuda(const int pillar_count, int* x_coors, int* y_coors, float* pfe_output, float* scattered_feature);
};



class ScatterCudaV2{
private:
    const int NUM_THREADS_;
    const int FEATURE_NUM_;
    const int GRID_X_SIZE_;
    const int GRID_Y_SIZE_;

public:
    /**
    * @brief Constructor
    * @param[in] NUM_THREADS The number of threads to launch cuda kernel
    * @param[in] MAX_NUM_PILLARS Maximum number of pillars
    * @param[in] GRID_X_SIZE Number of pillars in x-coordinate
    * @param[in] GRID_Y_SIZE Number of pillars in y-coordinate
    * @details Captital variables never change after the compile
    */
    ScatterCudaV2(const int NUM_THREADS, const int MAX_NUM_PILLARS, const int GRID_X_SIZE, const int GRID_Y_SIZE);

    /**
    * @brief Call scatter cuda kernel
    * @param[in] pillar_count The valid number of pillars
    * @param[in] x_coors X-coordinate indexes for corresponding pillars
    * @param[in] y_coors Y-coordinate indexes for corresponding pillars
    * @param[in] pfe_output Output from Pillar Feature Extractor
    * @param[out] scattered_feature Gridmap representation for pillars' feature
    * @details Allocate pillars in gridmap based on index(coordinates) information
    */
    void doScatterCuda(const int pillar_count,  int* coors, float* pfe_output, float* scattered_feature);
};


class ScatterMultiHeadsCuda {
private:
    const int num_threads_;
    const int grid_x_size_;
    const int grid_y_size_;

public:
    /**
     * @brief Constructor
     * @param[in] num_threads The number of threads to launch cuda kernel
     * @param[in] grid_x_size Number of pillars in x-coordinate
     * @param[in] grid_y_size Number of pillars in y-coordinate
     * @details Captital variables never change after the compile
     */
    ScatterMultiHeadsCuda(const int num_threads, const int grid_x_size,
                const int grid_y_size);

    /**
     * @brief Call scatter cuda kernel
     * @param[in] pillar_count The valid number of pillars
     * @param[in] x_coors X-coordinate indexes for corresponding pillars
     * @param[in] y_coors Y-coordinate indexes for corresponding pillars
     * @param[in] pfe_output Output from Pillar Feature Extractor
     * @param[out] scattered_feature Gridmap representation for pillars' feature
     * @details Allocate pillars in gridmap based on index(coordinates)
     * information
     */
    void DoScatterCuda(const int pillar_count, int* x_coors, int* y_coors,
                       float* pfe_output, float* scattered_feature);
};


