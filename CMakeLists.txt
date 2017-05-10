#-------------------------------------------------------------------------//
#
# Copyright 2017 Sascha Kaden
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
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++11")
add_definitions(-D_USE_MATH_DEFINES)

option(BUILD_FCL "Build FCL collision lib")
option(BUILD_EXAMPLES "Build the examples")
option(BUILD_TESTS "Build tests" ON)
option(BUILD_BENCHMARK "Build optional benchmark classes")

# external libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
find_package(assimp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PQP REQUIRED)
find_package(Threads REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include
        ${ASSIMP_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${PQP_INCLUDE_DIR})
SET(LIBRARIES
        ${PQP_LIBRARIES}
        ${ASSIMP_LIBRARIES})

# source files
ADD_SUBDIRECTORY(include)
ADD_SUBDIRECTORY(source)

# options
IF (${BUILD_FCL})
    find_package(fcl REQUIRED)
    find_package(ccd QUIET)
    include_directories(${FCL_INCLUDE_DIRS})
    LIST(APPEND LIBRARIES ${FCL_LIBRARIES} ${CCD_LIBRARIES})
ENDIF()
IF (${BUILD_EXAMPLES})
    ADD_SUBDIRECTORY(examples)
ENDIF()
IF (${BUILD_TESTS})
    enable_testing ()
    ADD_SUBDIRECTORY(tests)
ENDIF()

if (UNIX)
    find_package(Threads REQUIRED)
    set(append ${CMAKE_CXX_FLAGS} "-pthread")
    LIST(APPEND LIBRARIES pthread)
endif()

add_library(${PROJECT_NAME} STATIC ${SRCS})
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
install(DIRECTORY ${EIGEN3_INCLUDE_DIR}/Eigen/ DESTINATION Eigen)
install(DIRECTORY ${PQP_INCLUDE_DIR}/ DESTINATION PQP)
