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
#include <pthread.h>
#include <string>
#include <thread>
#include <iostream>

class Thread {
public:
    Thread(bool joinable = false, const std::string& name = "Thread")
            : joinable_(joinable), thread_name_(name) {}

    pthread_t Tid() const { return tid_; }

    void SetJoinable(bool joinable) {
        if (!started_) {
            joinable_ = joinable;
        }
    }

    void Start();

    void Join();

    bool IsAlive();

    std::string thread_name() const { return thread_name_; }
    void set_thread_name(const std::string& name) { thread_name_ = name; }

protected:
    virtual void Run() = 0;

    static void* ThreadRunner(void* arg) {
        Thread* t = reinterpret_cast<Thread*>(arg);
        t->Run();
        return nullptr;
    }

    pthread_t tid_ = 0;
    bool started_ = false;
    bool joinable_ = false;
    std::string thread_name_;
};
