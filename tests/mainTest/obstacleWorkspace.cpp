//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include <pathPlanner/NormalRRTPlanner.hpp>
#include <pathPlanner/PRMPlanner.hpp>
#include <pathPlanner/StarRRTPlanner.hpp>
#include <robot/PointRobot.h>
#include <core/utility/heuristic/HeuristicL1.hpp>
#include <core/utility/heuristic/HeuristicInf.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL1.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL2.hpp>
#include <core/utility/heuristic/HeuristicWeightVecInf.hpp>

using namespace rmpl;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(obstacleWorkspace) {
    const unsigned int dim = 2;
    Logging::setLogLevel(LogLevel::none);

    std::vector<std::shared_ptr<Heuristic<dim>>> heuristics;
    heuristics.push_back(std::make_shared<Heuristic<dim>>(Heuristic<dim>()));
    heuristics.push_back(std::make_shared<HeuristicL1<dim>>(HeuristicL1<dim>()));
    heuristics.push_back(std::make_shared<HeuristicInf<dim>>(HeuristicInf<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecL2<dim>>(HeuristicWeightVecL2<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecL1<dim>>(HeuristicWeightVecL1<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecInf<dim>>(HeuristicWeightVecInf<dim>()));

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
    robot->set2DWorkspace(workspace);
    std::shared_ptr<Planner<dim>> planner;

    Vector2 start(5, 5);
    Vector2 goal(95, 95);

    BOOST_TEST_CHECKPOINT("Options order: sampler, sampling, edgeHeuristic");
    for (auto heuristic : heuristics) {
        for (auto sampler : samplerMethods) {
            for (auto sampling : samplingStrategies) {
                PRMOptions<dim> prmOptions(30, 1, sampler, sampling, heuristic);
                PRMPlanner<dim> prmPlanner (robot, prmOptions);
                BOOST_TEST_CHECKPOINT("Calling PRM planning with (SamplerMethod | SamplingStrategy | EdgeHeuristic) =" << sampler << " | " << sampling << " | ");
                prmPlanner.computePath(start, goal, 100, 1);

                RRTOptions<dim> rrtOptions(30, 1, sampler, sampling, heuristic);
                NormalRRTPlanner<dim> normalRRTPlanner(robot, rrtOptions);
                BOOST_TEST_CHECKPOINT("Calling normal RRT planning with (SamplerMethod | SamplingStrategy | EdgeHeuristic) =" << sampler << " | " << sampling << " | " );
                normalRRTPlanner.computePath(start, goal, 100, 1);

                StarRRTPlanner<dim> starRRTPlanner(robot, rrtOptions);
                BOOST_TEST_CHECKPOINT("Calling RRT* planning with (SamplerMethod | SamplingStrategy | EdgeHeuristic) =" << sampler << " | " << sampling << " | " );
                starRRTPlanner.computePath(start, goal, 100, 1);
            }
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
