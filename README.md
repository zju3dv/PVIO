> [Robust and Efficient Visual-Inertial Odometry with Multi-plane Priors](http://www.cad.zju.edu.cn/home/gfzhang/projects/prcv2019-planeVIO.pdf)  
> [Jinyu Li](https://github.com/itsuhane), [Bangbang Yang](https://github.com/ybbbbt), [Kai Huang](https://github.com/elegracer), [Guofeng Zhang](https://github.com/guofengzhang), and Hujun Bao*   
> PRCV 2019, LNCS 11859, pp. 283–295, 2019.   

## How to use

For compilation:

* Install the dependencies: Eigen, Ceres Solver and OpenCV.
* Clone the repository.
* Build with `mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release  .. && make -j8`, you will need a compiler supporting C++17.
* Tested in Ubuntu 18.04 (with GCC 9.0 and CMake 3.11), and macOS 10.14.

For execution:
* `./pvio-pc [data_scheme]://[data_path] [config_yaml_path]`
  * e.g.
    * For EuRoC Dataset: `build/pvio-pc/pvio-pc euroc:///Data/EuRoC/V1_01_easy/mav0 config/euroc.yaml`
    * For TUM-VI Dataset: `build/pvio-pc/pvio-pc tum:///Data/TUM_VI/dataset-room1_512_16/mav0 config/tum_vi.yaml`
* The trajectory will be written in `trajectory.tum`.

## Publication

If you use this source code for your academic publication, please cite the following paper.
```
@inproceedings{PRCV-LiYHZB2019,
  author={Jinyu Li and Bangbang Yang and Kai Huang and Guofeng Zhang and Hujun Bao},
  title     = {Robust and Efficient Visual-Inertial Odometry with Multi-plane Priors},
  booktitle = {Pattern Recognition and Computer Vision - Second Chinese Conference,
               {PRCV} 2019, Xi'an, China, November 8-11, 2019, Proceedings, Part {III}},
  series    = {Lecture Notes in Computer Science},
  volume    = {11859},
  pages     = {283--295},
  publisher = {Springer},
  year      = {2019}
}
```

## Acknowledgements

This work is affliated with ZJU-SenseTime Joint Lab of 3D Vision, and its intellectual property belongs to SenseTime Group Ltd.

## Copyright
```
Copyright (c) ZJU-SenseTime Joint Lab of 3D Vision. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
