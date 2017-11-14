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

#include <ippp/modules/sampler/GridSampler.hpp>
#include <ippp/modules/sampler/SamplerNormalDist.hpp>
#include <ippp/modules/sampler/SamplerRandom.hpp>
#include <ippp/modules/sampler/SamplerUniform.hpp>
#include <ippp/modules/sampling/BridgeSampling.hpp>
#include <ippp/modules/sampling/GaussianDistSampling.hpp>
#include <ippp/modules/sampling/GaussianSampling.hpp>
#include <ippp/modules/sampling/MedialAxisSampling.hpp>
#include <ippp/modules/sampling/SamplingNearObstacle.hpp>
#include <ippp/modules/sampling/StraightSampling.hpp>

#include <ippp/environment/model/ModelFactoryFcl.h>
#include <ippp/environment/robot/MobileRobot.h>
#include <ippp/modules/collisionDetection/CollisionDetectionFcl.hpp>
#include <ippp/modules/trajectoryPlanner/LinearTrajectory.hpp>
#include <ippp/util/Utility.h>

double min = -5;
double max = 5;

using namespace ippp;

template <unsigned int dim>
void testSampling(const std::shared_ptr<Sampling<dim>> &sampling) {
    for (size_t i = 0; i < 10; ++i) {
        double number = sampling->getRandomNumber();
        EXPECT_TRUE(number >= 0);
        EXPECT_TRUE(number <= 1);

        auto config = sampling->getSample();
        if (util::empty<dim>(config))
            continue;
        for (unsigned int index = 0; index < dim; ++index) {
            EXPECT_TRUE(config[index] >= min);
            EXPECT_TRUE(config[index] <= max);
        }

        size_t sampleAmount = 10;
        auto samples = sampling->getSamples(sampleAmount);
        for (auto &config : samples) {
            if (util::empty<dim>(config))
                continue;
            for (unsigned int index = 0; index < dim; ++index) {
                EXPECT_TRUE(config[index] >= min);
                EXPECT_TRUE(config[index] <= max);
            }
        }
    }
}

template <unsigned int dim>
void createSampling() {
    std::vector<DofType> dofTypes;
    Vector<dim> minBound, maxBound;
    for (size_t i = 0; i < dim; ++i) {
        dofTypes.push_back(DofType::volumetricPos);
        minBound[i] = min;
        maxBound[i] = max;
    }

    std::shared_ptr<MobileRobot> robot(new MobileRobot(dim, std::make_pair(minBound, maxBound), dofTypes));
    ModelFactoryFcl factory;
    robot->setBaseModel(factory.createModel("assets/robotModels/2dLine.obj"));
    std::shared_ptr<Environment> environment(new Environment(3, AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));
    std::shared_ptr<CollisionDetection<dim>> collision(new CollisionDetectionFcl<dim>(environment));
    std::shared_ptr<TrajectoryPlanner<dim>> trajectory(new LinearTrajectory<dim>(collision, environment, 0.1));

    std::vector<std::shared_ptr<Sampler<dim>>> samplers;
    samplers.push_back(std::make_shared<SamplerRandom<dim>>(environment));
    samplers.push_back(std::make_shared<SamplerNormalDist<dim>>(environment));
    samplers.push_back(std::make_shared<SamplerUniform<dim>>(environment));
    if (dim < 4)
        samplers.push_back(std::make_shared<GridSampler<dim>>(environment));

    std::vector<std::shared_ptr<Sampling<dim>>> samplings;
    for (auto sampler : samplers) {
        samplings.push_back(std::make_shared<BridgeSampling<dim>>(environment, collision, trajectory, sampler, 1));
        samplings.push_back(std::make_shared<GaussianDistSampling<dim>>(environment, collision, trajectory, sampler, 1));
        samplings.push_back(std::make_shared<GaussianSampling<dim>>(environment, collision, trajectory, sampler, 1));
        samplings.push_back(std::make_shared<MedialAxisSampling<dim>>(environment, collision, trajectory, sampler, 1));
        samplings.push_back(std::make_shared<SamplingNearObstacle<dim>>(environment, collision, trajectory, sampler, 1));
        samplings.push_back(std::make_shared<StraightSampling<dim>>(environment, collision, trajectory, sampler, 1));
    }

    for (auto &sampling : samplings)
        testSampling(sampling);
}

TEST(SAMPLER, sample) {
    Logging::setLogLevel(LogLevel::info);

    createSampling<2>();
    createSampling<3>();
    createSampling<4>();
    createSampling<5>();
    createSampling<6>();
    createSampling<7>();
    createSampling<8>();
    createSampling<9>();
}
