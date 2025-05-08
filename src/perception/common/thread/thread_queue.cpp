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

#include "thread_queue.h"

template <typename T>
bool SafeQueue<T>::empty() {  //队列是否为空

    std::unique_lock<std::mutex> lock(m_mutex); //互斥信号变量加锁，防止m_queue被改变

    return m_queue.empty();
}

template <typename T>
int SafeQueue<T>::size() {
    std::unique_lock<std::mutex> lock(m_mutex); //互斥信号变量加锁，防止m_queue被改变

    return m_queue.size();
}

template <typename T>
void SafeQueue<T>::enqueue(T& t) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(t);
}

template <typename T>
bool SafeQueue<T>::dequeue(T& t) {
    std::unique_lock<std::mutex> lock(m_mutex); //队列加锁

    if (m_queue.empty()) {
        return false;
    }
    t = std::move(m_queue.front()); //取出队首元素，返回队首元素值，并进行右值引用

    m_queue.pop(); //弹出入队的第一个元素

    return true;
}