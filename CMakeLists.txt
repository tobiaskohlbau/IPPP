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
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-D_USE_MATH_DEFINES)

option(BUILD_EXAMPLES "Build the examples")
option(BUILD_TESTS "Build tests" ON)
option(BUILD_BENCHMARK "Build optional benchmark classes")

# external libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake ${PROJECT_SOURCE_DIR}/../IPPP_third_party)
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../IPPP_third_party)
find_package(assimp CONFIG REQUIRED)
find_package(fcl REQUIRED)
find_package(ccd CONFIG QUIET)
find_package(Eigen3 CONFIG REQUIRED)
find_package(PQP REQUIRED)
find_package(Threads REQUIRED)
set (NLOHMANN_JSON_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/nlohmann-json/src/)

SET(LIBRARIES
        ${PQP_LIBRARIES}
        ${ASSIMP_LIBRARIES}
        ${FCL_LIBRARIES}
        ${CCD_LIBRARIES})

# source files
ADD_SUBDIRECTORY(include/ippp)
ADD_SUBDIRECTORY(src)

if (UNIX)
    find_package(Threads REQUIRED)
    set(append ${CMAKE_CXX_FLAGS} "-pthread")
    LIST(APPEND LIBRARIES pthread)

endif()

# options
IF (${BUILD_EXAMPLES})
    ADD_SUBDIRECTORY(examples)
ENDIF()
IF (${BUILD_TESTS})
    enable_testing ()
    ADD_SUBDIRECTORY(tests)
ENDIF()

add_library(${PROJECT_NAME} ${SRCS})
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} ${LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_SOURCE_DIR}/include
        ${ASSIMP_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIRS}
        ${FCL_INCLUDE_DIRS}
        ${PQP_INCLUDE_DIR}
        ${NLOHMANN_JSON_INCLUDE_DIR})

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
install(DIRECTORY ${EIGEN3_INCLUDE_DIR}/Eigen/ DESTINATION Eigen)
install(DIRECTORY ${PQP_INCLUDE_DIR}/ DESTINATION PQP)