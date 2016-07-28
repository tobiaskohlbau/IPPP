#-------------------------------------------------------------------------//
#
# Copyright 2016 Sascha Kaden
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#-------------------------------------------------------------------------//

set(EXAMPLE_PLANNER_DIR ${CMAKE_SOURCE_DIR}/examples/Planner)

find_package( OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/examples/Planner)

set(source
    ${EXAMPLE_PLANNER_DIR}/main.cpp
    ${EXAMPLE_PLANNER_DIR}/ui/Drawing.cpp
    ${EXAMPLE_PLANNER_DIR}/ui/Drawing.cpp)

ADD_EXECUTABLE(Planner ${source})
TARGET_LINK_LIBRARIES(Planner RobotMotionPlanning PQP ${OpenCV_LIBS})

install(DIRECTORY ${EXAMPLE_PLANNER_DIR}/spaces/ DESTINATION Release/spaces)
install(DIRECTORY ${EXAMPLE_PLANNER_DIR}/meshes/ DESTINATION Release/meshes)
install(DIRECTORY ${EXAMPLE_PLANNER_DIR}/spaces/ DESTINATION Debug/spaces)
install(DIRECTORY ${EXAMPLE_PLANNER_DIR}/meshes/ DESTINATION Debug/meshes)