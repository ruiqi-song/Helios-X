//
// Created by ricky on 2022/2/25.
//

#pragma once

#include <map>
#include <memory>
#include <functional>
#include <typeindex>
#include <iostream>

namespace {

// The base type that is stored in the collection.
    struct Func_t {
        virtual ~Func_t() = default;
    };
// The map that stores the callbacks.
    using callbacks_t = std::map<std::type_index, std::unique_ptr<Func_t>>;
    callbacks_t callbacks;

// The derived type that represents a callback.
    template<typename ...A>
    struct Cb_t : public Func_t {
        using cb = std::function<void(A...)>;
        cb callback;
        Cb_t(cb p_callback) : callback(p_callback) {}
    };

// Wrapper function to call the callback stored at the given index with the
// passed argument.
    template<typename ...A>
    void call(std::type_index index, A&& ... args)
    {
        using func_t = Cb_t<A...>;
        using cb_t = std::function<void(A...)>;
        const Func_t& base = *callbacks[index];
        const cb_t& fun = static_cast<const func_t&>(base).callback;
        fun(std::forward<A>(args)...);
    }

} // end anonymous namespace
