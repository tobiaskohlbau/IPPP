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

#include <gtest/gtest.h>

#include <ippp/modules/collisionDetection/CollisionDetectionPqp.hpp>
#include <ippp/modules/trajectoryPlanner/LinearTrajectory.hpp>
#include <ippp/util/Utility.h>
#include <ippp/environment/robot/MobileRobot.h>

using namespace ippp;

TEST(TRAJECTORY, computeTrajectory) {
    // init TrajectoryPlanner
    const unsigned int dim = 6;
    std::vector<DofType> dofTypes = {DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                     DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos};
    std::shared_ptr<MobileRobot> robot(new MobileRobot(
        dim, std::make_pair(util::Vecd(-10, -10, -10, -10, -10, -10), util::Vecd(10, 10, 10, 10, 10, 10)), dofTypes));
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionPqp<dim>(environment));
    std::shared_ptr<TrajectoryPlanner<dim>> planner(new LinearTrajectory<dim>(collision, environment, 0.1));

    // test trajectories
    Vector6 init = util::Vecd(0, 0, 0, 0, 0, 0);
    Vector6 goal = util::Vecd(0, 0, 0, 0, 0, 0);
    std::vector<Vector6> path;
    path = planner->calcTrajectoryCont(init, goal);
    EXPECT_EQ(path.size(), 0);

    goal = util::Vecd(1, 1, 1, 1, 1, 1);
    path = planner->calcTrajectoryCont(init, goal);
    double dist = 1 / goal.norm() * 0.1;
    for (double i = 0; i < path.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            EXPECT_TRUE(path[i][j] <= dist + 0.0001);
            EXPECT_TRUE(path[i][j] >= dist - 0.0001);
        }
        dist += 1 / goal.norm() * 0.1;
    }

    goal = util::Vecd(-1, -1, -1, -1, -1, -1);
    path = planner->calcTrajectoryCont(init, goal);
    dist = -1 / goal.norm() * 0.1;
    for (double i = 0; i < path.size(); ++i) {
        for (int j = 0; j < 6; ++j) {
            EXPECT_TRUE(path[i][j] <= dist + 0.0001);
            EXPECT_TRUE(path[i][j] >= dist - 0.0001);
        }
        dist -= 1 / goal.norm() * 0.1;
    }
}
