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
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11 -pthread")

OPTION(BUILD_EXAMPLES "Build the examples")
OPTION(BUILD_TESTS "Build tests" ON)
OPTION(BUILD_FCL "Build FCL collision lib")

# external libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(assimp)
#find_package(Boost COMPONENTS thread REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PQP REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include
        ${assimp_INCLUDE_DIRS}/../
        ${EIGEN3_INCLUDE_DIR}
        ${PQP_INCLUDE_DIR})
#        ${Boost_INCLUDE_DIRS}

# source files
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(source)

add_definitions(-D_USE_MATH_DEFINES)

# options
IF (${BUILD_FCL})
    find_package(fcl REQUIRED)
    include_directories(${FCL_INCLUDE_DIRS})
ENDIF()
IF (${BUILD_EXAMPLES})
    ADD_SUBDIRECTORY(examples)
ENDIF()
IF (${BUILD_TESTS})
    enable_testing ()
    ADD_SUBDIRECTORY(tests)
ENDIF()

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} ${PQP_LIBRARIES} ${ASSIMP_LIBRARIES})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
install(DIRECTORY ${EIGEN3_INCLUDE_DIR}/Eigen/ DESTINATION Eigen)
install(DIRECTORY ${PQP_INCLUDE_DIR}/ DESTINATION PQP)
