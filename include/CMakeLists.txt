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
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Edge.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Graph.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Node.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/PointList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetection.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetection2D.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetectionPqp.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetectionTriangleRobot.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/sampler/Sampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/sampler/SamplerUniform.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/sampler/SamplerNormalDist.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/sampling/Sampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/sampling/SamplingNearObstacle.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/TrajectoryPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/Heuristic.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/HeuristicL1.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/HeuristicInf.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/HeuristicWeightVecL1.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/HeuristicWeightVecL2.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/heuristic/HeuristicWeightVecInf.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/KDNode.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilGeo.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilVec.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PlannerOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PRMOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/RRTOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/NormalRRTPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/Planner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/PRMPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTStarPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/RobotBase.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/robot/SerialRobot.hpp)

IF (${BUILD_FCL})
    list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetectionFcl.hpp)
ENDIF()

IF (${BUILD_BENCHMARK})
    list(APPEND SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/core/module/collisionDetection/CollisionDetectionPqpBenchmark.hpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)