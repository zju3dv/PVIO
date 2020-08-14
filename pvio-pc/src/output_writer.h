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
#ifndef PVIO_PC_OUTPUT_WRITER_H
#define PVIO_PC_OUTPUT_WRITER_H

#include <fstream>
#include <iostream>
#include <iomanip>
#include <pvio/pvio.h>

class OutputWriter {
  public:
    virtual ~OutputWriter() = default;
    virtual void write_pose(const double &t, const pvio::OutputPose &pose) = 0;
};

class TumOutputWriter : public OutputWriter {
    std::ofstream file;

  public:
    TumOutputWriter(const std::string &filename) {
        file.open(filename.c_str());
        if (!file.is_open()) {
            std::cerr << "Cannot open file " << std::quoted(filename) << std::endl;
        }
        file.precision(15);
    }

    ~TumOutputWriter() = default;

    void write_pose(const double &t, const pvio::OutputPose &pose) override {
        file << t << " " << pose.p.x() << " " << pose.p.y() << " " << pose.p.z() << " "
             << pose.q.x() << " " << pose.q.y() << " " << pose.q.z() << " " << pose.q.w() << "\n";
        file.flush();
    }
};

#endif // PVIO_PC_OUTPUT_WRITER_H
