/**************************************************************************
* This file is part of PVIO
*
* Copyright (c) ZJU-SenseTime Joint Lab of 3D Vision. All Rights Reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#ifndef PVIO_COMMON_H
#define PVIO_COMMON_H

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <array>
#include <atomic>
#include <bitset>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <forward_list>
#include <iomanip>
#include <limits>
#include <list>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include <memory>
#include <pvio/pvio.h>

#ifdef PVIO_DEBUG
#include <fstream>
#include <iostream>
#include <pvio/utility/debug.h>
#endif

#define PVIO_GRAVITY_NOMINAL 9.80665

namespace pvio {

template <typename T>
using map = Eigen::Map<T>;

template <typename T>
using const_map = Eigen::Map<const T>;

inline constexpr size_t nil() {
    return size_t(-1);
}

template <typename T>
struct compare; /*
    constexpr bool operator()(const T &a, const T &b) const;
*/

template <typename T>
struct compare<T *> {
    constexpr bool operator()(const T *a, const T *b) const {
        return std::less<T>()(*a, *b);
    }
};

template <class FlagEnum>
struct Flagged {
    static const size_t flag_num = static_cast<size_t>(FlagEnum::FLAG_NUM);

    Flagged() {
        flags.reset();
    }

    bool operator==(const Flagged &rhs) const {
        return (flags == rhs.flags);
    }

    bool flag(FlagEnum f) const {
        return flags[static_cast<size_t>(f)];
    }

    typename std::bitset<flag_num>::reference flag(FlagEnum f) {
        return flags[static_cast<size_t>(f)];
    }

    bool any_of(std::initializer_list<FlagEnum> flags) const {
        return std::any_of(flags.begin(), flags.end(), [this](FlagEnum f) { return flag(f); });
    }

    bool all_of(std::initializer_list<FlagEnum> flags) const {
        return std::all_of(flags.begin(), flags.end(), [this](FlagEnum f) { return flag(f); });
    }

    bool none_of(std::initializer_list<FlagEnum> flags) const {
        return std::none_of(flags.begin(), flags.end(), [this](FlagEnum f) { return flag(f); });
    }

  private:
    std::bitset<flag_num> flags;
};

struct ImuData {
    double t;
    vector<3> w;
    vector<3> a;
};

} // namespace pvio

#define synchronized(obj_ptr) if constexpr (auto local_synchronized_lock__ = (obj_ptr)->lock(); true)

#endif // PVIO_COMMON_H
