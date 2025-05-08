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

class SharedData {
public:
    SharedData() {}
    virtual ~SharedData() {}

   // virtual bool Init() = 0;

    // this api should clear all the memory used,
    // and would be called by SharedDataManager when reset DAGStreaming.
    virtual void Reset() { std::cout <<  "reset() not implemented." << std::endl;}

    virtual void RemoveStaleData() {
        std::cout << "remove_stale_data() not implemented." << std::endl;
    }

    virtual std::string name() const = 0;

private:

};