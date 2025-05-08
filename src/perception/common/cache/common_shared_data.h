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

#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <iostream>
#include <limits>

#include <deque>
#include <vector>
#include <string>
#include <mutex>
#include <functional>

#include "shared_data.h"

template <class M>
using SharedDataPtr = std::shared_ptr<M const>;

template <class T>
class CommonSharedData : SharedData{
public:
    //typedef std::shared_ptr<T const > SharedDataPtr;
    CommonSharedData(unsigned int cache_size = 20);
    virtual  ~CommonSharedData(){}

    void setCacheSize(unsigned int cache_size);
    void add(const SharedDataPtr<T> &msg);
    SharedDataPtr<T> getMsgBeforeTime(const ros::Time& time);
    SharedDataPtr<T> getMsgAfterTime(const ros::Time& time);
    SharedDataPtr<T> getMsgNearestTime(const ros::Time & time);
    SharedDataPtr<T> getMsgWithId(const int & id);

    SharedDataPtr<T> getMsgNowTime();
    std::deque<SharedDataPtr<T>>  getAllMsg();




    virtual  std::string name() const = 0;

    int size(){
        return cache_.size();
    }


private:
    mutable std::mutex cache_lock_;
    std::deque<SharedDataPtr<T>> cache_;
    unsigned int cache_size_;


};


template <class T>
CommonSharedData<T>::CommonSharedData(unsigned int cache_size) {
    setCacheSize(cache_size);
}

template <class T>
void CommonSharedData<T>::setCacheSize(unsigned int cache_size) {
    if(cache_size == 0){
        return;
    }
    cache_size_ = cache_size;
}

template <class T>
void CommonSharedData<T>::add(const SharedDataPtr<T> &msg) {
    std::lock_guard<std::mutex> lock(cache_lock_);
    while (cache_.size() >= cache_size_){
        cache_.pop_front();
    }

    cache_.push_back(msg);
}

template <class T>
SharedDataPtr<T> CommonSharedData<T>::getMsgBeforeTime(const ros::Time &time) {
    std::lock_guard<std::mutex> lock(cache_lock_);
    SharedDataPtr<T> out;
    unsigned int i = 0;
    int elem_index = -1;
    while(i < cache_.size() && cache_[i]->timestamp < time.toSec()){
        elem_index = i;
        i++;
    }

    if (elem_index >= 0){
        out = cache_[elem_index];
    }else{
        out = cache_[0];
    }

//    for(auto it = 0; it < cache_.size(); it++){
//        std::cout << std::to_string(cache_[it]->timestamp) << " " << std::to_string(cache_[it]->data.header.stamp) << std::endl;
//    }

    return out;

}

template <class T>
SharedDataPtr<T> CommonSharedData<T>::getMsgNearestTime(const ros::Time & time){
    std::lock_guard<std::mutex> lock(cache_lock_);
    SharedDataPtr<T> out;
    unsigned int i = 0;
    int elem_index = -1;

    while(i < cache_.size() && cache_[i]->timestamp < time.toSec()){
        elem_index = i;
        i++;
    }

    if (elem_index >= 0){
        if(elem_index < (cache_.size() - 1)){
            if((time.toSec() - cache_[elem_index]->timestamp) > (cache_[elem_index + 1]->timestamp - time.toSec())){
                out = cache_[elem_index + 1];
            } else{
                out = cache_[elem_index];
            }
        }else{
            out = cache_[elem_index];
        }
    }else{
        out = cache_[0];
    }
    return out;
}

template <class T>
SharedDataPtr<T> CommonSharedData<T>::getMsgAfterTime(const ros::Time &time) {
    std::lock_guard<std::mutex> lock(cache_lock_);
    SharedDataPtr<T> out;

    int i = static_cast<int>(cache_.size()) - 1;
    int elem_index = -1;
    while(i >= 0 &&  cache_[i]->timestamp > time.toSec()){
        elem_index = i;
        i--;
    }

    if(elem_index >= 0){
        out = cache_[elem_index];
    }
    else{
        out = cache_[0];
    }

    return out;


}

template <class T>
SharedDataPtr<T> CommonSharedData<T>::getMsgWithId(const int &id) {
    std::lock_guard<std::mutex> lock(cache_lock_);
    SharedDataPtr<T> out;

    if(id > 0){out =  cache_[id - 1];}
    else{out = cache_[0];}

    return  out;


}

template <class T>
SharedDataPtr<T> CommonSharedData<T>::getMsgNowTime(){
    std::lock_guard<std::mutex> lock(cache_lock_);
    SharedDataPtr<T> out;
    unsigned int i = 0;
    int elem_index = 0;
    if (cache_.size() >= 0){
        out = cache_[elem_index];
    }
    return out;

}

template <class T>
std::deque<SharedDataPtr<T>> CommonSharedData<T>::getAllMsg(){
    std::lock_guard<std::mutex> lock(cache_lock_);
    if (cache_.size() >= 0){
        return cache_;
    }
}


