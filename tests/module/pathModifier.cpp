//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/environment/robot/MobileRobot.h>
#include <ippp/modules/sampler/GridSampler.hpp>
#include <ippp/modules/sampler/SamplerNormalDist.hpp>
#include <ippp/modules/sampler/SamplerRandom.hpp>
#include <ippp/modules/sampler/SamplerUniform.hpp>
#include <ippp/util/UtilGeo.hpp>
#include <ippp/util/UtilVec.hpp>

double min = -5;
double max = 5;

using namespace ippp;

template <unsigned int dim>
void testSampler(const std::shared_ptr<Sampler<dim>> &sampler) {
    for (size_t i = 0; i < 10; ++i) {
        auto config = sampler->getSample();
        EXPECT_FALSE(util::empty<dim>(config));
        for (unsigned int index = 0; index < dim; ++index) {
            EXPECT_TRUE(config[index] >= min);
            EXPECT_TRUE(config[index] <= max);
        }

        double number = sampler->getRandomNumber();
        EXPECT_TRUE(number >= 0);
        EXPECT_TRUE(number <= 1);

        double angle = sampler->getRandomAngle();
        EXPECT_TRUE(angle >= 0);
        EXPECT_TRUE(angle <= util::twoPi());

        auto ray = sampler->getRandomRay();
        for (unsigned int index = 0; index < dim; ++index) {
            EXPECT_TRUE(ray[index] >= 0);
            EXPECT_TRUE(ray[index] <= 1);
        }
    }
}

template <unsigned int dim>
void createSampler() {
    std::vector<DofType> dofTypes;
    Vector<dim> minBound, maxBound;
    for (size_t i = 0; i < dim; ++i) {
        dofTypes.push_back(DofType::volumetricPos);
        minBound[i] = min;
        maxBound[i] = max;
    }

    std::shared_ptr<MobileRobot> robot(new MobileRobot(dim, std::make_pair(minBound, maxBound), dofTypes));
    std::shared_ptr<Environment> environment(new Environment(AABB(Vector3(-200, -200, -200), Vector3(200, 200, 200)), robot));

    std::vector<std::shared_ptr<Sampler<dim>>> samplers;
    samplers.push_back(std::make_shared<SamplerRandom<dim>>(environment));
    samplers.push_back(std::make_shared<SamplerNormalDist<dim>>(environment));
    samplers.push_back(std::make_shared<SamplerUniform<dim>>(environment));
    if (dim < 4)
        samplers.push_back(std::make_shared<GridSampler<dim>>(environment));

    for (auto &sampler : samplers)
        testSampler(sampler);
}

TEST(SAMPLER, sample) {
    createSampler<2>();
    createSampler<3>();
    createSampler<4>();
    createSampler<5>();
    createSampler<6>();
    createSampler<7>();
    createSampler<8>();
    createSampler<9>();
}
