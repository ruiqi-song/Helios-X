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

#include "inference_factory.h"

Inference *CreateInferenceEngine(const std::string &name,
                                 const std::string &conf_file,
                                 const std::string &weight_file,
                                 const std::vector<std::string> &outputs,
                                 const std::vector<std::string> &inputs,
                                 const bool &isBuildEngine){

    if (name == "TensorRTNet")
    {
        return new TensorRTNet(weight_file, isBuildEngine);
    }
    else if (name == "LibTorchNet")
    {
        return new LibTorchNet(conf_file, weight_file, outputs, inputs);
    }
//    auto libtorch = new LibTorchNet(conf_file, weight_file, outputs, inputs);
//    libtorch->set_gpu_id(0);
    return nullptr;

}

