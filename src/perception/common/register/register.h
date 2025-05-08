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
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include "container/any.h"

BaseClassMap &GlobalFactoryMap();

bool GetRegisteredClasses(
        const std::string &base_class_name,
        std::vector<std::string> *registered_derived_classes_names);

#define REGISTER_REGISTERER(base_class)                               \
  class base_class##Registerer {                                      \
    typedef ::Any Any;                                                \
    typedef ::FactoryMap FactoryMap;                                    \
                                                                      \
   public:                                                            \
    static base_class *GetInstanceByName(const ::std::string &name) { \
      FactoryMap &map = GlobalFactoryMap()[#base_class];              \
      FactoryMap::iterator iter = map.find(name);                     \
      if (iter == map.end()) {                                        \
        for (auto c : map) {                                          \
          std::cout << "Instance:" << c.first;                        \
        }                                                             \
        std::cout << "Get instance " << name << " failed.";           \
        return nullptr;                                               \
      }                                                               \
      Any object = iter->second->NewInstance();                      \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static std::vector<base_class *> GetAllInstances() {              \
      std::vector<base_class *> instances;                            \
      FactoryMap &map = GlobalFactoryMap()[#base_class];              \
      instances.reserve(map.size());                                  \
      for (auto item : map) {                                         \
        Any object = item.second->NewInstance();                     \
        instances.push_back(*(object.AnyCast<base_class *>()));       \
      }                                                               \
      return instances;                                               \
    }                                                                 \
    static const ::std::string GetUniqInstanceName() {                \
      FactoryMap &map = GlobalFactoryMap()[#base_class];              \
                                                                      \
      return map.begin()->first;                                      \
    }                                                                 \
    static base_class *GetUniqInstance() {                            \
      FactoryMap &map = GlobalFactoryMap()[#base_class];              \
      Any object = map.begin()->second->NewInstance();                \
      return *(object.AnyCast<base_class *>());                       \
    }                                                                 \
    static bool IsValid(const ::std::string &name) {                  \
      FactoryMap &map = GlobalFactoryMap()[#base_class];              \
      return map.find(name) != map.end();                             \
    }                                                                 \
  };

#define REGISTER_CLASS(clazz, name)                                           \
class ObjectFactory##name : public ObjectFactory {                          \
   public:                                                                    \
    virtual ~ObjectFactory##name() {}                                         \
    virtual Any NewInstance() {                                       \
      return Any(new name());                                         \
    }                                                                         \
  };                                                                          \
  inline void RegisterFactory##name() {                                       \
    FactoryMap &map = GlobalFactoryMap()[#clazz];                     \
    if (map.find(#name) == map.end()) map[#name] = new ObjectFactory##name(); \
  }



