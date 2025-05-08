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

#include "inference/inference.h"

class LibTorchNet : public Inference {
public:
    LibTorchNet(const std::string &conf_file,
                const std::string &weight_file,
                const std::vector<std::string> &outputs,
                const std::vector<std::string> &inputs){};


    virtual ~LibTorchNet(){};



    bool Init(const std::map<std::string, std::vector<int>> &shapes) override;

    void Build() override;

    void Infer(float *input, float *output, int batchSize, char* inputName, char* outputName) override;
    void Infer(std::vector<float> &input, std::vector<int> &output, int batchSize, char* inputName, char* outputName) override;


};
