# IPPP -- Interchangeable Probalistic Path Planning

[![Build Status](https://travis-ci.org/SaschaKaden/IPPP.svg?branch=master)](https://travis-ci.org/SaschaKaden/IPPP)
[![Build status](https://ci.appveyor.com/api/projects/status/fxqb3k2b4csn0gur/branch/master?svg=true)](https://ci.appveyor.com/project/SaschaKaden/ippp/branch/master)
[![Coverity Status](https://scan.coverity.com/projects/9839/badge.svg)](https://scan.coverity.com/projects/saschakaden-ippp)

IPPP has the target to provide a coherent interchangeable for path planning with a serial or mobile robots.
All important modules can be changed from the user.

## Robots
Currently available robots:
* [Kinova Jaco](http://www.kinovarobotics.com/service-robotics/products/robot-arms/)
* KukaKR5
* 2D point and 2D triangle robot, for test cases

All robots are based on a container interface, therefore the user can build his own robots.


## Planning algorithm
Currently available algorithms:
* RRT
* RRT*
* PRM
* EST
* SRT


## Dependencies
* [FCL](https://github.com/flexible-collision-library/fcl)(Flexible Collision Library) for collision detection between 3d meshes
* [Assimp](http://www.assimp.org)(Open Asset Import Library) for import and export of meshes (cad models)


#### Dependencies of the examples
* [OpenCV](http://opencv.org/) for manipulation and illustration of 2D examples
* [QT](https://www.qt.io/) for the user interface


## Getting Started
Small step for step instructions for Ubuntu.

### Dependencies
* `sudo apt-get install cmake libeigen3-dev libccd-dev liboctomap-dev libboost-all-dev curl tar`
* `curl -L https://github.com/assimp/assimp/archive/v4.0.1.tar.gz | tar zx`
* `cd assimp-4.0.1 && curl -L https://goo.gl/UMB94G | patch -p0 assimp-config.cmake.in`
* `mkdir build && cd build`
* `cmake .. -DASSIMP_BUILD_TESTS=OFF && cmake --build . -- -j`nproc` && cmake -DCMAKE_INSTALL_PREFIX=../../deps -P cmake_install.cmake`
* `cd ../..`
* `curl -L https://github.com/flexible-collision-library/fcl/archive/0.5.0.tar.gz | tar zx`
* `mkdir -p fcl-0.5.0/build && cd fcl-0.5.0/build`
* `cmake .. -DFCL_BUILD_TESTS=OFF && cmake --build . -- -j`nproc` && cmake -DCMAKE_INSTALL_PREFIX=../../deps -P cmake_install.cmake`
* `cd ../..`

### IPP
* `git clone https://github.com/SaschaKaden/IPPP.git`
* `cd IPPP && git submodule update --init --recursive`
* `mkdir build && cd build`
* `cmake .. -DBUILD_EXAMPLES=ON -DCMAKE_PREFIX_PATH=../deps && cmake --build . -- -j`nproc``

In the examples directory are small commented Getting Started projects.

## License
Copyright © 2017 Sascha Kaden
Licensed under the Apache 2.0.
