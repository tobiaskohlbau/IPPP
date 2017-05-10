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

#include <core/module/collisionDetection/CollisionDetection2D.hpp>
#include <core/module/sampler/SamplerNormalDist.hpp>
#include <core/module/sampler/SamplerUniform.hpp>
#include <core/module/sampling/SamplingNearObstacle.hpp>
#include <core/utility/heuristic/HeuristicInf.hpp>
#include <core/utility/heuristic/HeuristicL1.hpp>
#include <core/utility/heuristic/HeuristicWeightVecInf.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL1.hpp>
#include <core/utility/heuristic/HeuristicWeightVecL2.hpp>
#include <pathPlanner/PRM.hpp>
#include <pathPlanner/RRTStar.hpp>
#include <robot/PointRobot.h>

using namespace ippp;

BOOST_AUTO_TEST_SUITE(constructor)

BOOST_AUTO_TEST_CASE(clearWorkspace) {
    const unsigned int dim = 2;
    Logging::setLogLevel(LogLevel::none);

    std::vector<std::shared_ptr<Heuristic<dim>>> heuristics;
    heuristics.push_back(std::make_shared<Heuristic<dim>>(Heuristic<dim>()));
    heuristics.push_back(std::make_shared<HeuristicL1<dim>>(HeuristicL1<dim>()));
    heuristics.push_back(std::make_shared<HeuristicInf<dim>>(HeuristicInf<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecL2<dim>>(HeuristicWeightVecL2<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecL1<dim>>(HeuristicWeightVecL1<dim>()));
    heuristics.push_back(std::make_shared<HeuristicWeightVecInf<dim>>(HeuristicWeightVecInf<dim>()));

    Eigen::MatrixXi workspace = Eigen::MatrixXi::Constant(100, 100, 250);
    Vector2 minBoundary(0.0, 0.0);
    Vector2 maxBoundary(workspace.rows(), workspace.cols());
    std::shared_ptr<PointRobot> robot(new PointRobot(minBoundary, maxBoundary));
    std::shared_ptr<ModelContainer> model(new Model2D(workspace));
    robot->setWorkspace(model);
    std::shared_ptr<CollisionDetection<2>> collision(new CollisionDetection2D(robot));
    std::shared_ptr<TrajectoryPlanner<2>> trajectory(new TrajectoryPlanner<2>(1.5, collision));
    std::shared_ptr<Planner<dim>> planner;

    std::vector<std::shared_ptr<Sampler<dim>>> sampler;
    sampler.push_back(std::shared_ptr<Sampler<dim>>(new Sampler<dim>(robot)));
    sampler.push_back(std::shared_ptr<Sampler<dim>>(new SamplerUniform<dim>(robot)));
    sampler.push_back(std::shared_ptr<Sampler<dim>>(new SamplerNormalDist<dim>(robot)));
    std::vector<std::shared_ptr<Sampling<2>>> samplings;
    for (auto samp : sampler) {
        samplings.push_back(std::shared_ptr<Sampling<dim>>(new Sampling<2>(robot, collision, trajectory, samp)));
        samplings.push_back(std::shared_ptr<Sampling<dim>>(new SamplingNearObstacle<2>(robot, collision, trajectory, samp)));
    }

    Vector2 start(5, 5);
    Vector2 goal(95, 95);

    BOOST_TEST_CHECKPOINT("Options order: sampler, sampling, edgeHeuristic");
    for (auto heuristic : heuristics) {
        for (auto sampling : samplings) {
            PRMOptions<dim> prmOptions(30, collision, trajectory, sampling, heuristic);
            PRMPlanner<dim> prmPlanner(robot, prmOptions);
            BOOST_TEST_CHECKPOINT("Calling PRM planning");
            prmPlanner.computePath(start, goal, 500, 2);

            RRTOptions<dim> rrtOptions(30, collision, trajectory, sampling, heuristic);
            RRT<dim> normalRRTPlanner(robot, rrtOptions);
            BOOST_TEST_CHECKPOINT("Calling normal RRT planning");
            normalRRTPlanner.computePath(start, goal, 500, 2);

            RRTStarPlanner<dim> starRRTPlanner(robot, rrtOptions);
            BOOST_TEST_CHECKPOINT("Calling RRT* planning");
            starRRTPlanner.computePath(start, goal, 500, 2);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
