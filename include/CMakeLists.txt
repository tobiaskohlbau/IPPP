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
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Edge.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Graph.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/KDNode.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Node.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/PointList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetection.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetection2D.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionPqp.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionTriangleRobot.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/DistanceMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/L1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/InfMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightVecL1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightVecL2Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightVecInfMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/BruteForceNF.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/NeighborFinder.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/Sampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerLinear.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerRotatory.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerNormalDist.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerUniform.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SeedSampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/Sampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/SamplingNearObstacle.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/trajectoryPlanner/TrajectoryPlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilGeo.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilIO.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilVec.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/UtilCollision.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PlannerOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PRMOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/RRTOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/SRTOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/Planner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/PRM.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRT.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTStar.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTStarContTraj.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/SRT.hpp)

IF (${BUILD_FCL})
    list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionFcl.hpp)
ENDIF()

IF (${BUILD_BENCHMARK})
    list(APPEND SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionPqpBenchmark.hpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)
