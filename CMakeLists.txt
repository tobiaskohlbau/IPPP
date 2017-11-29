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

cmake_minimum_required(VERSION 3.2.2)
project(IPPP)

option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_BENCHMARK "Build optional benchmark classes" OFF)

# external libraries
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)
list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../IPPP_third_party)
#todo: workaround for the findassimp (no more difference of windows and linux because of the cmake of assimp)
if (UNIX)
    find_package (assimp REQUIRED)
else (UNIX)
    find_package (assimp CONFIG REQUIRED)
endif ()
find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

# workarround: generated cmake config does not provide library target
add_library(assimp::assimp UNKNOWN IMPORTED)
set_target_properties(assimp::assimp PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${ASSIMP_INCLUDE_DIRS}")
if(EXISTS "${ASSIMP_LIBRARIES}")
    set_target_properties(assimp::assimp PROPERTIES IMPORTED_LOCATION "${ASSIMP_LIBRARIES}")
endif()

# worarround: fcl required ccd but dos not pull dependency by it's own
find_package(ccd REQUIRED)
set_target_properties(fcl PROPERTIES INTERFACE_LINK_LIBRARIES ccd)

add_library(json::json INTERFACE IMPORTED)
set_target_properties(json::json PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/third_party/nlohmann-json/src")

add_library(gsl::gsl INTERFACE IMPORTED)
set_target_properties(gsl::gsl PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/third_party/GSL/include")

add_subdirectory(third_party/PQP
    ${CMAKE_BINARY_DIR}/third_party/PQP
    EXCLUDE_FROM_ALL)

# copy assets
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory
             ${CMAKE_SOURCE_DIR}/assets
             ${CMAKE_CURRENT_BINARY_DIR}/assets/)

# generate library
add_library(${PROJECT_NAME}
    src/Identifier.cpp
    src/util/Logging.cpp
    src/statistic/Statistics.cpp
    src/statistic/StatisticCollector.cpp
    src/statistic/StatisticCountCollector.cpp
    src/statistic/StatisticContainer.cpp
    src/statistic/StatisticSizeTContainer.cpp

    src/environment/cad/CadDrawing.cpp
    src/environment/cad/CadImportExport.cpp
    src/environment/cad/CadProcessing.cpp
    src/environment/Environment.cpp
    src/environment/robot/Jaco.cpp
    src/environment/robot/Joint.cpp
    src/environment/robot/KukaKR5.cpp
    src/environment/robot/MobileRobot.cpp
    src/environment/robot/PointRobot.cpp
    src/environment/robot/RobotBase.cpp
    src/environment/robot/SerialRobot.cpp
    src/environment/robot/SerialRobot2D.cpp
    src/environment/robot/TriangleRobot2D.cpp
    src/environment/model/ModelContainer.cpp
    src/environment/model/ModelFactoryPqp.cpp
    src/environment/model/ModelFactoryTriangle2D.cpp
    src/environment/model/ModelFactoryFcl.cpp
    src/environment/model/ModelFcl.cpp
    src/environment/model/ModelPqp.cpp
    src/environment/model/ModelTriangle2D.cpp
    src/environment/model/PointModel.cpp

    src/ui/Configurator.cpp
    src/ui/EnvironmentConfigurator.cpp
)

target_compile_features(${PROJECT_NAME}
    PUBLIC
        cxx_auto_type
        cxx_defaulted_functions
        cxx_lambdas
        cxx_nullptr
        cxx_nullptr
        cxx_override
        cxx_rvalue_references
        cxx_right_angle_brackets)

target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        Eigen3::Eigen
        json::json
        assimp::assimp
        fcl
        PQP)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-pthread" pthread)
if(pthread)
    find_package(Threads REQUIRED)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
            "-pthread")
    target_link_libraries(${PROJECT_NAME}
        PRIVATE
            pthread)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${CMAKE_SOURCE_DIR}/include)

# options
if(${BUILD_EXAMPLES})
    add_subdirectory(examples)
endif()
if(${BUILD_TESTS})
    # Prevent overriding the parent project's compiler/linker
    # settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/third_party/googletest
        ${CMAKE_BINARY_DIR}/third_party/googletest
        EXCLUDE_FROM_ALL
    )
    if(WIN32)
        # Prevent deprecated warnings
        # https://docs.microsoft.com/en-us/cpp/error-messages/compiler-warnings/compiler-warning-level-3-c4996
        target_compile_definitions(gtest PRIVATE _CRT_SECURE_NO_WARNINGS)
        target_compile_definitions(gtest PRIVATE _CRT_NONSTDC_NO_WARNINGS)
        target_compile_definitions(gtest_main PRIVATE _CRT_SECURE_NO_WARNINGS)
        target_compile_definitions(gtest_main PRIVATE _CRT_NONSTDC_NO_WARNINGS)
    endif()

    enable_testing()
    add_subdirectory(tests)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION "lib")
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)
install(FILES ${PROJECT_SOURCE_DIR}/third_party/nlohmann-json/src/json.hpp DESTINATION include)
