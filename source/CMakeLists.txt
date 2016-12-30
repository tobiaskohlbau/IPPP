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

list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Edge.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Node.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/CollisionDetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/ModuleBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/Sampler.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/Sampling.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/TrajectoryPlanner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/Logging.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/Utility.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilGeo.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilList.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilVec.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/RRTOptions.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PlannerOptions.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PRMOptions.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/GenericRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/Jaco.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/Joint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/KukaKR5.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/MeshContainer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/MobileRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/PointRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/RobotBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/SerialRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/TriangleRobot2D.cpp)

set(SRCS "${SRCS}" PARENT_SCOPE)