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

cmake_minimum_required (VERSION 3.2.2)
project (IPPP)
set (CMAKE_AUTOMOC ON)
add_definitions (-D_USE_MATH_DEFINES)

option (BUILD_EXAMPLES "Build the examples" OFF)
option (BUILD_TESTS "Build tests" ON)
option (BUILD_BENCHMARK "Build optional benchmark classes" OFF)

# external libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
if (UNIX)
    list (APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../IPPP_third_party)
    find_package (assimp REQUIRED)
    find_package (ccd REQUIRED)
 else (UNIX)
    find_package (assimp CONFIG REQUIRED)
    find_package (ccd CONFIG REQUIRED)
endif ()
find_package (fcl REQUIRED)
find_package (Eigen3 CONFIG REQUIRED)
find_package (PQP REQUIRED)
find_package (Threads REQUIRED)
set (NLOHMANN_JSON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/nlohmann-json/src/)

set(LIBRARIES
        ${ASSIMP_LIBRARIES}
        ${CCD_LIBRARIES}
        ${FCL_LIBRARIES}
        ${PQP_LIBRARIES})

if (UNIX)
    find_package (Threads REQUIRED)
    set (append ${CMAKE_CXX_FLAGS} "-pthread")
    list (APPEND LIBRARIES pthread)
endif ()

# source files
ADD_SUBDIRECTORY (include/ippp)
ADD_SUBDIRECTORY (src)

# options
IF (${BUILD_EXAMPLES})
    ADD_SUBDIRECTORY (examples)
ENDIF ()
IF (${BUILD_TESTS})
    ADD_SUBDIRECTORY (tests)
ENDIF ()

# generate library
add_library (${PROJECT_NAME} ${SRCS})
set_property (TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries (${PROJECT_NAME} ${LIBRARIES})
target_include_directories (${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${ASSIMP_INCLUDE_DIR}
        ${CCD_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${NLOHMANN_JSON_INCLUDE_DIR}
        ${PQP_INCLUDE_DIR})

install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)