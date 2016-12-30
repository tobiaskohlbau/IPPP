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
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/KDNode.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/PointList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/NormalRRTPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/Planner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/PRMPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/StarRRTPlanner.hpp)

set(SRCS "${SRCS}" PARENT_SCOPE)