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
#ifndef PVIO_FACTOR_H
#define PVIO_FACTOR_H

#include <pvio/common.h>

namespace pvio {

class Frame;
class Track;

class Factor {
    Factor() = delete;
    struct factor_construct_t {};

  public:
    struct FactorCostFunction {
        virtual ~FactorCostFunction() = default;
        virtual void update() = 0;
    };

    static std::unique_ptr<Factor> create_marginalization_error(const matrix<> &sqrt_inv_cov, const vector<> &infovec, std::vector<Frame *> &&frames);
    static std::unique_ptr<Factor> create_reprojection_error(Track *track, Frame *frame, size_t keypoint_index);
    static std::unique_ptr<Factor> create_preintegration_error(Frame *frame_i, Frame *frame_j);

    template <typename T>
    T *get_cost_function() {
        return static_cast<T *>(cost_function.get());
    }

    Factor(std::unique_ptr<FactorCostFunction> cost_function, const factor_construct_t &);
    virtual ~Factor();

  private:
    std::unique_ptr<FactorCostFunction> cost_function;
};

} // namespace pvio

#endif // PVIO_FACTOR_H
