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
project (RobotMotionPlanning)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

OPTION(BUILD_EXAMPLES "Build the examples")
OPTION(DEBUG_OUTPUT "Display debug output")
OPTION(BUILD_TESTS "Build tests")

# Eigen
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# PQP
set(PQP_DIR ${CMAKE_SOURCE_DIR}/thirdParty/PQP_v1.3 CACHE PATH "PQP DIR")
include_directories(${PQP_DIR}/include)
link_directories(${PQP_DIR}/lib)

# Assimp
set(Assimp_SOURCE_DIR ${CMAKE_SOURCE_DIR}/../assimp CACHE PATH "Assimp source DIR")
set(Assimp_BINARY_DIR /usr/local CACHE PATH "Assimp binary DIR")
INCLUDE_DIRECTORIES(${Assimp_SOURCE_DIR}/include ${Assimp_SOURCE_DIR}/code)
LINK_DIRECTORIES(${Assimp_BINARY_DIR} ${Assimp_BINARY_DIR}/lib)

# fcl
find_package(fcl REQUIRED)
include_directories(${FCL_INCLUDE_DIRS})

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SRCS
    include/core/KDNode.hpp
    include/core/KDTree.hpp
    include/core/Vec.hpp
    source/core/CollisionDetection.cpp
    source/core/Edge.cpp
    source/core/Graph.cpp
    source/core/List.cpp
    source/core/Logging.cpp
    source/core/ModuleBase.cpp
    source/core/Node.cpp
    source/core/Sampling.cpp
    source/core/TrajectoryPlanner.cpp
    source/core/Utilities.cpp
    source/pathPlanner/NormalRRTPlanner.cpp
    source/pathPlanner/Planner.cpp
    source/pathPlanner/PlannerOptions.cpp
    source/pathPlanner/PRMPlanner.cpp
    source/pathPlanner/PRMOptions.cpp
    source/pathPlanner/RRTPlanner.cpp
    source/pathPlanner/RRTOptions.cpp
    source/pathPlanner/StarRRTPlanner.cpp
    source/robot/GenericRobot.cpp
    source/robot/Jaco.cpp
    source/robot/Joint.cpp
    source/robot/KukaKR5.cpp
    source/robot/MeshContainer.cpp
    source/robot/MobileRobot.cpp
    source/robot/PointRobot.cpp
    source/robot/RobotBase.cpp
    source/robot/SerialRobot.cpp)

IF (${DEBUG_OUTPUT})
    add_definitions(-DDEBUG_OUTPUT)
ENDIF()

IF (${BUILD_EXAMPLES})
    ADD_SUBDIRECTORY(examples)
ENDIF()

IF (${BUILD_TESTS})
    ADD_SUBDIRECTORY(unitTest)
ENDIF()

add_custom_target(myInstall make install)

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} pthread PQP assimp ${fcl_LIBRARIES} ccd)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
install(DIRECTORY ${EIGEN3_INCLUDE_DIR}/Eigen/ DESTINATION Eigen)
install(DIRECTORY ${PQP_DIR}/include/ DESTINATION PQP)
