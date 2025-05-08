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
#include <map>
#include<string>
#include<unordered_map>
#include<memory>
#include<functional>

#include "any.h"
#include "register/register.h"




template <class T>
class SingleIOCContainer {
public:
    SingleIOCContainer(void){}
    ~SingleIOCContainer()
    {

    }

    template<class D>
    void RegisterType(std::string strKey) {
        std::function<T* ()> function = [] {return new D();};
        RegisterType(strKey, function);
    }


    T* Resolve(std::string strKey) {
        if (m_createMap.find(strKey) == m_createMap.end())
            return nullptr;
        std::function<T* ()> function = m_createMap[strKey];
        return function();
    }


    std::shared_ptr<T> ResolveShared(std::string strKey) {
        T* ptr = Resolve(strKey);
        return std::shared_ptr<T>(ptr);
    }
private:
    void RegisterType(std::string strKey, std::function<T* ()> creator) {
        if (m_createMap.find(strKey) != m_createMap.end()) {
            throw std::invalid_argument("error");
        }
        m_createMap.emplace(strKey, creator);
    }

    std::map<std::string, std::function<T* ()>> m_createMap;
};





class IocContainer : NonCopyable
{
public:
    IocContainer(void){}
    ~IocContainer(void){}

    template<class T, typename Depend, typename... Args>
    void RegisterType(const std::string& strKey)
    {
        std::function<T* (Args...)> function = [](Args... args){ return new Depend(args...); };
        RegisterType(strKey, function);
    }

    template<class T, typename... Args>
    T* Resolve(const std::string& strKey, Args... args)
    {
        if (m_creatorMap.find(strKey) == m_creatorMap.end())
            return nullptr;

        Any resolver = m_creatorMap[strKey];
        std::function<T* (Args...)> function = resolver.AnyCastPtr<std::function<T* (Args...)>>();

        return function(args...);
    }

    template<class T, typename... Args>
    std::shared_ptr<T> ResolveShared(const std::string& strKey, Args... args)
    {
        T* t = Resolve<T>(strKey, args...);

        return std::shared_ptr<T>(t);
    }

private:
    void RegisterType(const std::string& strKey, Any constructor)
    {
        if (m_creatorMap.find(strKey) != m_creatorMap.end())
            throw std::invalid_argument("this key has already exist!");

        //通过Any擦除了不同类型的构造器
        m_creatorMap.emplace(strKey, constructor);
    }

private:
    std::unordered_map<std::string, Any> m_creatorMap;
};



