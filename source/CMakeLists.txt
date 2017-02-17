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
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/ModuleBase.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/Logging.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/Utility.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/Jaco.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/Joint.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/KukaKR5.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/model/CadProcessing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/model/ModelFactoryPqp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/model/ModelPqp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/PointRobot.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/TriangleRobot2D.cpp)

IF (${BUILD_FCL})
    list(APPEND SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/robot/model/ModelFactoryFcl.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/robot/model/ModelFcl.cpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)