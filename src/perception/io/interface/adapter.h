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

class BaseMessage{
public:
    virtual ~BaseMessage() = default;

    /**
     * @brief returns the topic name that this adapter listens to.
     */
    virtual const std::string& topic_name() const = 0;
};

template <typename T>
class AdapterMessage: public BaseMessage{
public:
    typedef T MessageType;
    typedef typename std::function<void(const T&)> Callback;

    const std::string& topic_name() const  { return topic_name_; }


    AdapterMessage(const std::string& adapter_name, const std::string& topic_name): topic_name_(topic_name){}

    /**
     *  @brief registers the provided callback function to the adapter,
     *  so that the callback function will be called once right after the
     *  message hits the adapter.
     *  @param callback the callback with signature void(const D &).
     */
    void AddCallback(Callback callback) {
        receive_callbacks_.push_back(callback);
    }

    /**
     * @brief proactively invokes the callbacks one by one registered with the
     * specified data.
     * @param data the specified data.
     */
    void StartCallbacks(const MessageType& data) {
        for (const auto& callback : receive_callbacks_) {
            callback(data);
        }
    }

private:

    /// The topic name that the adapter listens to.
    std::string topic_name_;
    /// User defined function when receiving a message
    std::vector<Callback> receive_callbacks_;

    friend class AdapterManager;
};
