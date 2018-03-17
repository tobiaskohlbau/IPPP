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

#include <ippp/Core.h>
#include <ippp/Environment.h>
#include <ippp/Planner.h>
#include <ippp/UI.h>

using namespace ippp;

TEST(MAIN, clearWorkspace2D) {
    Logging::setLogLevel(LogLevel::off);
    const unsigned int dim = 2;

    EnvironmentConfigurator environmentConfig;
    AABB workspaceBounding(Vector3(0, 0, 0), Vector3(100, 100, 100));
    environmentConfig.setWorkspaceProperties(workspaceBounding);
    environmentConfig.setRobotType(RobotType::Point);
    auto environment = environmentConfig.getEnvironment();

    ModuleConfigurator<dim> modulConfig;
    modulConfig.setEnvironment(environment);
    modulConfig.setVadilityCheckerType(ValidityCheckerType::AlwaysValid);
    modulConfig.setSamplerProperties("asldkf2o345;lfdnsa;f", 1);

    std::vector<MetricType> metricTypes;
    std::vector<EvaluatorType> evalTypes;
    std::vector<NeighborType> neighborTypes;
    std::vector<PathModifierType> modifierTypes;
    std::vector<SamplerType> samplerTypes;
    std::vector<SamplingType> samplingTypes;
    std::vector<TrajectoryType> trajectoryTypes;

    metricTypes.push_back(MetricType::L1);
    metricTypes.push_back(MetricType::L2);
    metricTypes.push_back(MetricType::Inf);
    metricTypes.push_back(MetricType::L1Weighted);
    metricTypes.push_back(MetricType::L2Weighted);
    metricTypes.push_back(MetricType::InfWeighted);

    evalTypes.push_back(EvaluatorType::SingleIteration);
    //evalTypes.push_back(EvaluatorType::Time);
    //evalTypes.push_back(EvaluatorType::Query);
    //evalTypes.push_back(EvaluatorType::QueryOrTime);
    modulConfig.setEvaluatorProperties(10, 3);

    neighborTypes.push_back(NeighborType::BruteForce);
    neighborTypes.push_back(NeighborType::KDTree);

    modifierTypes.push_back(PathModifierType::Dummy);
    modifierTypes.push_back(PathModifierType::NodeCut);

    samplerTypes.push_back(SamplerType::Random);
    samplerTypes.push_back(SamplerType::Uniform);
    samplerTypes.push_back(SamplerType::UniformBiased);
    //samplerTypes.push_back(SamplerType::SamplerNormalDist);

    samplingTypes.push_back(SamplingType::Straight);
    samplingTypes.push_back(SamplingType::NearObstacle);
    //samplingTypes.push_back(SamplingType::Bridge);
    //samplingTypes.push_back(SamplingType::Gaussian);
    //samplingTypes.push_back(SamplingType::GaussianDist);

    trajectoryTypes.push_back(TrajectoryType::Linear);
    trajectoryTypes.push_back(TrajectoryType::RotateAtS);

    Vector2 start(5, 5);
    Vector2 goal(95, 95);

    for (auto &metricType : metricTypes) {
        modulConfig.setMetricType(metricType);
        for (auto &evalType : evalTypes) {
            modulConfig.setEvaluatorType(evalType);
            for (auto &neighborType : neighborTypes) {
                modulConfig.setNeighborFinderType(neighborType);
                for (auto &modifier : modifierTypes) {
                    modulConfig.setPathModifierType(modifier);
                    for (auto &sampler : samplerTypes) {
                        modulConfig.setSamplerType(sampler);
                        for (auto &sampling : samplingTypes) {
                            modulConfig.setSamplingType(sampling);
                            for (auto &trajectory : trajectoryTypes) {
                                modulConfig.setTrajectoryType(trajectory);

                                PRM<dim> prm(environment, modulConfig.getPRMOptions(15), modulConfig.getGraph());
                                EXPECT_TRUE(prm.computePath(start, goal, 300, 1));
                                modulConfig.resetModules();
                                RRT<dim> rrt(environment, modulConfig.getRRTOptions(15), modulConfig.getGraph());
                                EXPECT_TRUE(rrt.computePath(start, goal, 300, 1));
                                modulConfig.resetModules();
                                RRTStar<dim> rrtStar(environment, modulConfig.getRRTOptions(15), modulConfig.getGraph());
                                EXPECT_TRUE(rrtStar.computePath(start, goal, 300, 1));
                            }
                        }
                    }
                }
            }
        }
    }
}
