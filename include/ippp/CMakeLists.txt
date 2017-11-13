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
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionResult.h
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionRequest.h
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetection.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionAABB.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionAlwaysValid.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionFcl.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionPqp.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionSphere.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetection2D.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionTriangleRobot.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/dataObj/Edge.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dataObj/Graph.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dataObj/KDNode.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dataObj/Node.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dataObj/PointList.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/DistanceMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/L1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/L2Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/InfMetric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/WeightedL1Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/WeightedL2Metric.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/distanceMetrics/WeightedInfMetric.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/evaluator/ComposeEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/evaluator/Evaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/evaluator/QueryEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/evaluator/SingleIterationEvaluator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/evaluator/TimeEvaluator.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/neighborFinders/BruteForceNF.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/neighborFinders/KDTree.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/neighborFinders/NeighborFinder.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/pathModifier/DummyPathModifier.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/pathModifier/PathModifier.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/pathModifier/NodeCutPathModifier.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/GridSampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/Sampler.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/SamplerRandom.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/SamplerNormalDist.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/SamplerUniform.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampler/SeedSampler.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/BridgeSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/GaussianSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/GaussianDistSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/MedialAxisSampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/Sampling.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/SamplingNearObstacle.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/sampling/StraightSampling.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/modules/trajectoryPlanner/LinearTrajectory.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/trajectoryPlanner/RotateAtS.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/trajectoryPlanner/TrajectoryPlanner.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilCollision.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilGeo.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilIO.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilList.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilTrajectory.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilVec.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilEnvironment.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/MapGenerator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/UtilPlanner.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/environment/Environment.h

        ${CMAKE_CURRENT_SOURCE_DIR}/planner/options/PlannerOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/options/PRMOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/options/RRTOptions.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/options/SRTOptions.hpp

        ${CMAKE_CURRENT_SOURCE_DIR}/planner/EST.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/Planner.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/PRM.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/RRT.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/RRTStar.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/RRTStarContTraj.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/SRT.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/planner/TreePlanner.hpp
        
        ${CMAKE_CURRENT_SOURCE_DIR}/ui/Configurator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ui/EnvironmentConfigurator.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ui/ModuleConfigurator.hpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ui/Writer.hpp)

IF (${BUILD_BENCHMARK})
    list(APPEND SRCS
        ${CMAKE_CURRENT_SOURCE_DIR}/modules/collisionDetection/CollisionDetectionPqpBenchmark.hpp)
ENDIF()

set(SRCS "${SRCS}" PARENT_SCOPE)
