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

#include <memory>
#include <typeindex>
#include <exception>
#include <iostream>

class NonCopyable
{
public:
    NonCopyable(const NonCopyable&) = delete; // deleted
    NonCopyable& operator = (const NonCopyable&) = delete; // deleted
    NonCopyable() = default;   // available
};

class Any {
public:
    Any() : m_ptr(nullptr),content_(nullptr), m_tpIndex(std::type_index(typeid(void))) {}

    template<typename ValueType, class = typename std::enable_if
            <!std::is_same<typename std::decay<ValueType>::type, Any>::value,
                    ValueType>::type>
    explicit Any(const ValueType &value, int id)
            : m_ptr(nullptr), content_(new Holder<ValueType>(value)){}

    // loop
    Any(const Any &other) :
            m_ptr(other.Clone()),
            content_(other.content_ ? other.content_->clone() : nullptr),
            m_tpIndex(other.m_tpIndex){}

    Any(Any &&other) : m_ptr(std::move(other.m_ptr)), content_(nullptr), m_tpIndex(other.m_tpIndex) {}

    ~Any() { delete content_; }

    template<typename ValueType, class = typename std::enable_if
            <!std::is_same<typename std::decay<ValueType>::type, Any>::value,
                    ValueType>::type>
    Any(ValueType &&value) : m_ptr(
            new HolderPtr<typename std::decay<ValueType>::type>(std::forward<ValueType>(value))),
                             content_(new Holder<ValueType>(value)),
                             m_tpIndex(std::type_index(typeid(typename std::decay<ValueType>::type))) {}

    template<typename ValueType>
    ValueType *AnyCast() {
        return content_ ? &(static_cast<Holder <ValueType> *>(content_)->held_)
                        : nullptr;
    }

    template<class ValueType>
    ValueType &AnyCastPtr() {
        auto holder = dynamic_cast<HolderPtr <ValueType> *> (m_ptr.get());
        return holder->held_;
    }

    Any &operator=(const Any &a) {
        if (m_ptr == a.m_ptr)
            return *this;

        m_ptr = a.Clone();
        m_tpIndex = a.m_tpIndex;
        return *this;
    }

    Any &operator=(Any &&a) {
        if (m_ptr == a.m_ptr)
            return *this;

        m_ptr = std::move(a.m_ptr);
        m_tpIndex = a.m_tpIndex;
        return *this;
    }

private:
    class PlaceHolder {
    public:
        virtual ~PlaceHolder() {}

        virtual PlaceHolder *clone() const = 0;
    };

    template<typename ValueType>
    class Holder : public PlaceHolder {
    public:
        explicit Holder(const ValueType &value) : held_(value) {}

        virtual ~Holder() {}

        virtual PlaceHolder *clone() const { return new Holder(held_); }

        ValueType held_;
    };


    class PlaceHolderPtr;

    typedef std::unique_ptr<PlaceHolderPtr> BasePtr;

    class PlaceHolderPtr {
    public:
        virtual ~PlaceHolderPtr() {}

        virtual BasePtr Clone() const = 0;
    };

    template<typename ValueTypePtr>
    class HolderPtr : public PlaceHolderPtr {
    public:
        template<typename ValueType>
        HolderPtr(ValueType &&value) : held_(std::forward<ValueType>(value)) {}

        BasePtr Clone() const {
            return BasePtr(new HolderPtr<ValueTypePtr>(held_));
        }

        ValueTypePtr held_;
    };


    BasePtr Clone() const {
        if (m_ptr != nullptr) {
            return m_ptr->Clone();
        }
        return nullptr;
    }

    BasePtr m_ptr;
    PlaceHolder *content_;
    std::type_index m_tpIndex;

    //FRIEND_TEST(ObjectFactoryTest, test_ObjectFactory);
};

#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);

    class ObjectFactory {
    public:
        ObjectFactory() {}

        virtual ~ObjectFactory() {}

        virtual Any NewInstance() { return Any(); }

    private:
    DISALLOW_COPY_AND_ASSIGN(ObjectFactory);

    };

typedef std::unordered_map<std::string, ObjectFactory *> FactoryMap;
typedef std::unordered_map<std::string, FactoryMap> BaseClassMap;




