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



cmake_minimum_required (VERSION 2.8.10)
project (RobotMotionPlanningLib)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()
set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

# external libaries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/include/vrep)

set(PQP_DIR "/home/sascha/projects/pqp" CACHE PATH "PQP DIR")
include_directories(${PQP_DIR}/include)
link_directories(${PQP_DIR}/lib)

set(SRCS
    core/CollisionDetection.cpp
    core/Base.cpp
    core/Edge.cpp
    core/Graph.cpp
    core/Node.cpp
    core/Sampling.cpp
    core/TrajectoryPlanner.cpp
    include/core/KDNode.hpp
    include/core/KDTree.hpp
    include/core/Vec.hpp
    planner/NormalRRTPlanner.cpp
    planner/Planner.cpp
    planner/RRTPlanner.cpp
    planner/StarRRTPlanner.cpp
    robot/CadFileLoader.cpp
    robot/Jaco.cpp
    robot/PointRobot.cpp
    robot/RobotBase.cpp
    vrep/extApi.c
    vrep/extApiPlatform.c
    vrep/Helper.cpp)

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} pthread PQP)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/lib)
install(DIRECTORY include DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
install(DIRECTORY ${EIGEN3_INCLUDE_DIR} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
install(DIRECTORY ${PQP_DIR}/include DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/PQP)
