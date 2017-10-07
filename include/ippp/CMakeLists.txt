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
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetection.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionAABB.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionAlwaysValid.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionPqp.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionSphere.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetection2D.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionTriangleRobot.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/CollisionData.h
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Edge.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Graph.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/KDNode.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/Node.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/dataObj/PointList.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/DistanceMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/L1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/L2Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/InfMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightedL1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightedL2Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/distanceMetrics/WeightedInfMetric.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/evaluator/ComposeEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/evaluator/Evaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/evaluator/QueryEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/evaluator/SingleIterationEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/evaluator/TimeEvaluator.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/BruteForceNF.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/neighborFinders/NeighborFinder.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/pathModifier/DummyPathModifier.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/pathModifier/PathModifier.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/pathModifier/NodeCutPathModifier.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/Sampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerRandom.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerNormalDist.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SamplerUniform.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampler/SeedSampler.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/BridgeSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/GaussianSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/GaussianDistSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/MedialAxisSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/Sampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/SamplingNearObstacle.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/sampling/StraightSampling.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/trajectoryPlanner/LinearTrajectory.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/trajectoryPlanner/RotateAtS.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/trajectoryPlanner/TrajectoryPlanner.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilCollision.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilGeo.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilIO.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilTrajectory.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/core/util/UtilVec.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/environment/Environment.h
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/util/UtilEnvironment.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/environment/util/MapGenerator.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PlannerOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/PRMOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/RRTOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/options/SRTOptions.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/EST.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/Planner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/PRM.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRT.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTStar.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/RRTStarContTraj.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/SRT.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/TreePlanner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/pathPlanner/util/UtilPlanner.hpp)

IF (${BUILD_FCL})
    list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionFcl.hpp)
ENDIF()

IF (${BUILD_BENCHMARK})
    list(APPEND SRCS
            ${CMAKE_CURRENT_SOURCE_DIR}/core/collisionDetection/CollisionDetectionPqpBenchmark.hpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)
