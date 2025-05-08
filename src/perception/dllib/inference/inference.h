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
#include <vector>
#include <map>

class Inference {
public:
    Inference() = default;
    virtual ~Inference() = default;

    virtual void Build() = 0;

    virtual void Infer(float *input, float *output, int batchSize, char* inputName, char* outputName) = 0;

    virtual void Infer(std::vector<float> &input, std::vector<int> &output, int batchSize, char* inputName, char* outputName) = 0;

    virtual bool Init(const std::map<std::string, std::vector<int>> &shapes) = 0;

    void set_max_batch_size(const int &batch_size) { max_batch_size_ = batch_size; };

    void set_gpu_id(const int &gpu_id) { gpu_id_ = gpu_id; };

protected:
    int max_batch_size_ = 1;
    int gpu_id_ = 0;
};
