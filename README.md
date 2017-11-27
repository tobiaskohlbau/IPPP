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
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix manipulation 
* [PQP](http://gamma.cs.unc.edu/SSV/)(Proximity Query Package) for collision detection between 3d meshes
* [FCL](https://github.com/flexible-collision-library/fcl)(Flexible Collision Library) for collision detection between 3d meshes
* [Assimp](http://www.assimp.org)(Open Asset Import Library) for import and export of meshes (cad models)
* [JSON](https://github.com/nlohmann/json)
* [Googletest](https://github.com/google/googletest)
* [gflags](https://github.com/gflags/gflags)

#### Dependencies of the examples
* [OpenCV](http://opencv.org/) for manipulation and illustration of 2D examples
* [QT](https://www.qt.io/) for the user interface


## Getting Started
Small step for step instructions for Ubuntu.

* `sudo apt-get install cmake libeigen3-dev libboost-system-dev libboost-test-dev libboost-filesystem-dev`
* `git clone https://github.com/SaschaKaden/RobotMotionPlanner.git`
* `git clone https://github.com/SaschaKaden/RMPThirdParty.git`
* `cd RobotMotionPlanner && mkdir build && cd build && cmake -DPQP_ROOT_DIR="../../RMPThirdParty/PQP_v1.3"`

In the examples directory is a small commented Getting Started project.


## License
Copyright © 2017 Sascha Kaden
Licensed under the Apache 2.0.
