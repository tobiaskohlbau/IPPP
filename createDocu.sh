#!/bin/bash

git clone git@github.com:SaschaKaden/RobotMotionPlanner.git source
doxygen docs/Doxyfile
rm -rf source

echo "documentation is updated"
