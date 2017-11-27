//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

#include <boost/test/unit_test.hpp>

#include <ippp/modules/collisionDetection/CollisionDetection2D.hpp>
#include <ippp/modules/distanceMetrics/InfMetric.hpp>
#include <ippp/modules/distanceMetrics/L1Metric.hpp>
#include <ippp/modules/distanceMetrics/WeightedInfMetric.hpp>
#include <ippp/modules/distanceMetrics/WeightedL1Metric.hpp>
#include <ippp/modules/distanceMetrics/WeightedL2Metric.hpp>
#include <ippp/pathPlanner/NormalRRT.hpp>
#include <ippp/pathPlanner/PRM.hpp>
#include <ippp/pathPlanner/RRTStar.hpp>
#include <ippp/environment/PointRobot.h>

using namespace ippp;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(obstacleWorkspace) {
    const unsigned int dim = 2;
    Logging::setLogLevel(LogLevel::none);

    std::vector<std::shared_ptr<DistanceMetric<dim>>> metrics;
    metrics.push_back(std::make_shared<DistanceMetric<dim>>(DistanceMetric<dim>()));
    metrics.push_back(std::make_shared<L1Metric<dim>>(L1Metric<dim>()));
    metrics.push_back(std::make_shared<InfMetric<dim>>(InfMetric<dim>()));
    metrics.push_back(std::make_shared<WeightVecL2Metric<dim>>(WeightVecL2Metric<dim>()));
    metrics.push_back(std::make_shared<WeightVecL1Metric<dim>>(WeightVecL1Metric<dim>()));
    metrics.push_back(std::make_shared<WeightVecInfMetric<dim>>(WeightVecInfMetric<dim>()));

    std::vector<SamplerMethod> samplerMethods;
    samplerMethods.push_back(SamplerMethod::randomly);
    samplerMethods.push_back(SamplerMethod::uniform);
    samplerMethods.push_back(SamplerMethod::standardDistribution);

    std::vector<SamplingStrategy> samplingStrategies;
    samplingStrategies.push_back(SamplingStrategy::normal);
    samplingStrategies.push_back(SamplingStrategy::nearObstacles);

    // create obstacle workspace;
    Eigen::MatrixXi workspace = Eigen::MatrixXi::Constant(100, 100, 250);
    for (int i = 20; i < 50; ++i) {
        workspace(i, i) = 0;
        workspace(i + 10, 30) = 0;
    }

    Vector2 minBoundary(0.0, 0.0);
    Vector2 maxBoundary(workspace.rows(), workspace.cols());
    std::shared_ptr<PointRobot> robot(new PointRobot(minBoundary, maxBoundary));
    std::shared_ptr<ModelContainer> model(new Model2D(workspace));
    robot->setWorkspace(model);
    std::shared_ptr<CollisionDetection<2>> collision(new CollisionDetection2D(robot));
    std::shared_ptr<Planner<dim>> planner;

    Vector2 start(5, 5);
    Vector2 goal(95, 95);

    BOOST_TEST_CHECKPOINT("Options order: sampler, sampling, distanceMetric");
    for (auto metric : metrics) {
        for (auto sampler : samplerMethods) {
            for (auto sampling : samplingStrategies) {
                PRMOptions<dim> prmOptions(30, 1, collision, sampler, sampling, metric);
                PRMPlanner<dim> prmPlanner(robot, prmOptions);
                BOOST_TEST_CHECKPOINT("Calling PRM planning with (SamplerMethod | SamplingStrategy | distanceMetric) ="
                                      << sampler << " | " << sampling << " | ");
                prmPlanner.computePath(start, goal, 300, 1);

                RRTOptions<dim> rrtOptions(30, 1, collision, sampler, sampling, metric);
                RRT<dim> normalRRTPlanner(robot, rrtOptions);
                BOOST_TEST_CHECKPOINT("Calling normal RRT planning with (SamplerMethod | SamplingStrategy | distanceMetric) ="
                                      << sampler << " | " << sampling << " | ");
                normalRRTPlanner.computePath(start, goal, 300, 1);

                RRTStarPlanner<dim> starRRTPlanner(robot, rrtOptions);
                BOOST_TEST_CHECKPOINT("Calling RRT* planning with (SamplerMethod | SamplingStrategy | distanceMetric) ="
                                      << sampler << " | " << sampling << " | ");
                starRRTPlanner.computePath(start, goal, 300, 1);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
