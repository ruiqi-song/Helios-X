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
#include <mutex>
#include <queue>


template<typename T>
class SafeQueue {
private:
    std::queue<T> m_queue; //利用模板函数构造队列

    std::mutex m_mutex;//访问互斥信号量


public:
    SafeQueue() { //空构造函数
    }

    SafeQueue(SafeQueue &other) {//拷贝构造函数
        //TODO:
    }
    ~SafeQueue() { //析构函数
    }

    bool empty();
    int size();
//队列添加元素
    void enqueue(T &t);
//队列取出元素
    bool dequeue(T &t);
};
