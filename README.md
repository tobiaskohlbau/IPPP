# Robot Motion Planner Library

[![Build Status](https://travis-ci.org/SaschaKaden/RobotMotionPlanner.svg?branch=master)](https://travis-ci.org/SaschaKaden/RobotMotionPlanner)
[![Coverity Status](https://scan.coverity.com/projects/9839/badge.svg)](https://scan.coverity.com/projects/saschakaden-robotmotionplanner)

RMPL has the target to provide a coherent interface for path planning with a serial robot.

## Robots
Currently available robots:
* [Kinova Jaco](http://www.kinovarobotics.com/service-robotics/products/robot-arms/)
* KukaKR5
* 2D Point Robot, for test cases

The user can set up his own robot too, he has only to pass the D-H parameter and the triangle meshes (.obj).


## Planning algorithm
Currently available algorithms:
* RRT
* RRT*
* sPRM


## Dependencies
For matrix manipulation the [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) library is included.
Collision detection between triangle meshes by the [PQP](http://gamma.cs.unc.edu/SSV/) library and the [fcl](https://github.com/flexible-collision-library/fcl) library.
 
#### Dependencies of the examples
To show results, the [OpenCV](http://opencv.org/) library is for 2D examples included and the [QT](https://www.qt.io/) library is for the gui2D example included. 
The results from serial robots can be tested by the [vrep](http://www.coppeliarobotics.com/) simulation (an interface for the Jaco robot is written).


## License
Copyright © 2016 Sascha Kaden
Licensed under the Apache 2.0.
