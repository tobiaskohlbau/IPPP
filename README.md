# Robot Motion Planner Library

RMPL has the target to provide a coherent interface for path plannig with a serial robot.
For this the user can set up his own roboter or use a existing serial robot (e.g. Jaco).

Currently available robots:
* [Kinova Jaco](http://www.kinovarobotics.com/service-robotics/products/robot-arms/)
* 2D Point Robot, for test cases


For path planning the normal RRT and RRT* algorith is implemented.

The libary uses the extern libary Eigen for matrix manipulation.

For drawing the results the OpenCV libary is included.

## License
Copyright © 2016 Sascha Kaden
Licensed under the Apache 2.0.
