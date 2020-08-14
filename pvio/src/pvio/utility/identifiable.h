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
#ifndef PVIO_IDENTIFIABLE_H
#define PVIO_IDENTIFIABLE_H

#include <pvio/common.h>

namespace pvio {

template <typename T>
class Identifiable {
  public:
    size_t id() const {
        return id_value;
    }

    bool operator<(const T &other) const {
        return id_value < other.id_value;
    }

  protected:
    Identifiable() :
        Identifiable(generate_id()) {
    }

    Identifiable(size_t id_value) :
        id_value(id_value) {
    }

  private:
    static size_t generate_id() {
        static size_t s_id = 0;
        s_id++;
        if (s_id == nil()) { // make sure we never get nil(), this almost never happen in reality.
            s_id = 0;
        }
        return s_id;
    }

    const size_t id_value;
};

} // namespace pvio

#endif // PVIO_IDENTIFIABLE_H
