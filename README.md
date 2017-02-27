# Robot Motion Planner Library

[![Build Status](https://travis-ci.org/SaschaKaden/RobotMotionPlanner.svg?branch=master)](https://travis-ci.org/SaschaKaden/RobotMotionPlanner)
[![Coverity Status](https://scan.coverity.com/projects/9839/badge.svg)](https://scan.coverity.com/projects/saschakaden-robotmotionplanner)

RMPL has the target to provide a coherent interface for path planning with a serial robot.

## Robots
Currently available robots:
* [Kinova Jaco](http://www.kinovarobotics.com/service-robotics/products/robot-arms/)
* KukaKR5
* 2D point and 2D triangle robot, for test cases

The user can set up his own robot too, he has only to pass the D-H parameter and the triangle meshes (.obj).


## Planning algorithm
Currently available algorithms:
* RRT
* RRT*
* sPRM


## Dependencies
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) for matrix manipulation 
* [PQP](http://gamma.cs.unc.edu/SSV/)(Proximity Query Package) for collision detection between 3d meshes
* [Assimp](http://www.assimp.org)(Open Asset Import Library) for import and export of meshes
* [boost test framework](http://www.boost.org/doc/libs/1_63_0/libs/test/doc/html/index.html)
* optional [FCL](https://github.com/flexible-collision-library/fcl)(Flexible Collision Library) for collision detection between 3d meshes

#### Dependencies of the examples
* [OpenCV](http://opencv.org/) for manipulation and illustration of 2D examples
* [QT](https://www.qt.io/) for the user interface
* optional [vrep](http://www.coppeliarobotics.com/) simulation


## Getting Started
Small step for step instructions for Ubuntu.

* sudo apt-get install cmake
* sudo apt-get install libeigen3-dev
* sudo apt-get install libboost-system-dev
* sudo apt-get install libboost-test-dev
* sudo apt-get install libboost-filesystem-dev
* git clone https://github.com/SaschaKaden/RobotMotionPlanner.git
* git clone https://github.com/SaschaKaden/RMPThirdParty.git
* cd RobotMotionPlanner
* mkdir build
* cd build
* cmake -DPQP_ROOT_DIR="../../RMPThirdParty/PQP_v1.3"


## License
Copyright © 2017 Sascha Kaden
Licensed under the Apache 2.0.
