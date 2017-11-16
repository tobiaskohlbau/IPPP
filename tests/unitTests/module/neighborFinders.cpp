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

#include <ippp/modules/distanceMetrics/InfMetric.hpp>
#include <ippp/modules/distanceMetrics/L1Metric.hpp>
#include <ippp/modules/distanceMetrics/L2Metric.hpp>
#include <ippp/modules/distanceMetrics/WeightedInfMetric.hpp>
#include <ippp/modules/distanceMetrics/WeightedL1Metric.hpp>
#include <ippp/modules/distanceMetrics/WeightedL2Metric.hpp>
#include <ippp/modules/neighborFinders/BruteForceNF.hpp>
#include <ippp/modules/neighborFinders/KDTree.hpp>

using namespace ippp;

template <unsigned int dim>
void testConstructor() {
    std::vector<std::shared_ptr<DistanceMetric<dim>>> metrics;
    metrics.push_back(std::make_shared<L1Metric<dim>>());
    metrics.push_back(std::make_shared<L2Metric<dim>>());
    metrics.push_back(std::make_shared<InfMetric<dim>>());
    metrics.push_back(std::make_shared<WeightedL1Metric<dim>>());
    metrics.push_back(std::make_shared<WeightedL2Metric<dim>>());
    metrics.push_back(std::make_shared<WeightedInfMetric<dim>>());

    for (auto &metric : metrics) {
        auto finder1 = std::make_shared<KDTree<dim, std::shared_ptr<Node<dim>>>>(metric);
        auto finder2 = std::make_shared<BruteForceNF<dim, std::shared_ptr<Node<dim>>>>(metric);
    }
}

TEST(NEIGHBORFINDERS, constructor) {
    testConstructor<2>();
    testConstructor<3>();
    testConstructor<4>();
    testConstructor<5>();
    testConstructor<6>();
    testConstructor<7>();
    testConstructor<8>();
    testConstructor<9>();
}
