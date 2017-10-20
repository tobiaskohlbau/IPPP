//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the >License>);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an >AS IS> BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <ippp/core/Identifier.h>
#include <ippp/core/types.h>

#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/core/collisionDetection/CollisionDetection2D.hpp>
#include <ippp/core/collisionDetection/CollisionDetectionAABB.hpp>
#include <ippp/core/collisionDetection/CollisionDetectionAlwaysValid.hpp>
#include <ippp/core/collisionDetection/CollisionDetectionPqp.hpp>
#include <ippp/core/collisionDetection/CollisionDetectionSphere.hpp>
#include <ippp/core/collisionDetection/CollisionDetectionTriangleRobot.hpp>

#include <ippp/core/dataObj/Edge.hpp>
#include <ippp/core/dataObj/Graph.hpp>
#include <ippp/core/dataObj/Node.hpp>
#include <ippp/core/dataObj/PointList.hpp>

#include <ippp/core/distanceMetrics/DistanceMetric.hpp>
#include <ippp/core/distanceMetrics/InfMetric.hpp>
#include <ippp/core/distanceMetrics/L1Metric.hpp>
#include <ippp/core/distanceMetrics/L2Metric.hpp>
#include <ippp/core/distanceMetrics/WeightedInfMetric.hpp>
#include <ippp/core/distanceMetrics/WeightedL1Metric.hpp>
#include <ippp/core/distanceMetrics/WeightedL2Metric.hpp>

#include <ippp/core/evaluator/ComposeEvaluator.hpp>
#include <ippp/core/evaluator/Evaluator.hpp>
#include <ippp/core/evaluator/QueryEvaluator.hpp>
#include <ippp/core/evaluator/SingleIterationEvaluator.hpp>
#include <ippp/core/evaluator/TimeEvaluator.hpp>

#include <ippp/core/neighborFinders/BruteForceNF.hpp>
#include <ippp/core/neighborFinders/KDTree.hpp>
#include <ippp/core/neighborFinders/NeighborFinder.hpp>

#include <ippp/core/pathModifier/DummyPathModifier.hpp>
#include <ippp/core/pathModifier/NodeCutPathModifier.hpp>
#include <ippp/core/pathModifier/PathModifier.hpp>

#include <ippp/core/sampler/Sampler.hpp>
#include <ippp/core/sampler/SamplerNormalDist.hpp>
#include <ippp/core/sampler/SamplerRandom.hpp>
#include <ippp/core/sampler/SamplerUniform.hpp>
#include <ippp/core/sampler/SeedSampler.hpp>

#include <ippp/core/sampling/BridgeSampling.hpp>
#include <ippp/core/sampling/GaussianDistSampling.hpp>
#include <ippp/core/sampling/GaussianSampling.hpp>
#include <ippp/core/sampling/MedialAxisSampling.hpp>
#include <ippp/core/sampling/Sampling.hpp>
#include <ippp/core/sampling/SamplingNearObstacle.hpp>
#include <ippp/core/sampling/StraightSampling.hpp>

#include <ippp/core/trajectoryPlanner/LinearTrajectory.hpp>
#include <ippp/core/trajectoryPlanner/RotateAtS.hpp>

#include <ippp/core/statistic/StatisticCollector.h>
#include <ippp/core/statistic/StatisticContainer.h>
#include <ippp/core/statistic/Statistics.h>

#include <ippp/core/util/Logging.h>
#include <ippp/core/util/UtilGeo.hpp>
#include <ippp/core/util/UtilIO.hpp>
#include <ippp/core/util/UtilList.hpp>
#include <ippp/core/util/UtilVec.hpp>
