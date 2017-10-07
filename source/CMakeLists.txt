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

list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/core/Identifier.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/Logging.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/CadDrawing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/CadImportExport.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/CadProcessing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/Environment.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/Jaco.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/Joint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/KukaKR5.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/MobileRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/PointRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/RobotBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/SerialRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/robot/TriangleRobot2D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelContainer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelFactoryPqp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelFactoryTriangle2D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelPqp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelTriangle2D.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/PointModel.cpp)

IF (${BUILD_FCL})
    list(APPEND SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelFactoryFcl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/environment/model/ModelFcl.cpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)
