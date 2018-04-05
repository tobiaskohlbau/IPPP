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

#include "ConfigurationMA.h"

#include <ippp/statistic/Stats.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

ConfigurationMA::ConfigurationMA(RobotType robotType, bool useObstacle, bool useConstraint, bool isMobile)
    : m_propertyCollector(std::make_shared<StatsPropertyCollector>("Properties")) {
    Stats::addCollector(m_propertyCollector);
    auto seeds = getSeeds();
    std::vector<double> stepSizes = getSerialStepSizes();
    if (isMobile)
        stepSizes = getMobileStepSizes();
    std::vector<bool> optimizes = {false, true};
    std::vector<bool> useObstacles = {false};
    if (useObstacle)
        useObstacles.push_back(true);

    std::vector<SamplerType> samplers = {SamplerType::Uniform, SamplerType::UniformBiased};
    std::vector<SamplingType> samplings = {SamplingType::Straight, SamplingType::NearObstacle};
    if (useConstraint) {
        samplings.push_back(SamplingType::TS);
        samplings.push_back(SamplingType::FOR);
        samplings.push_back(SamplingType::Berenson);
    }
    std::vector<PathModifierType> modifierTypes = {PathModifierType::Dummy, PathModifierType::NodeCut};
    std::vector<PlannerType> rrtTypes = {PlannerType::RRTStar, PlannerType::AdaptedRRT, PlannerType::CiBRRT};

    for (auto &stepSize : stepSizes) {
        for (auto optimize : optimizes) {
            for (auto obstacle : useObstacles) {
                for (auto &sampler : samplers) {
                    for (auto &sampling : samplings) {
                        for (auto &modifierType : modifierTypes) {
                            for (auto &rrtType : rrtTypes) {
                                for (auto &seed : seeds) {
                                    ParamsMA param;
                                    param.robotType = robotType;
                                    param.optimize = optimize;
                                    param.pathModifier = modifierType;
                                    param.samplerType = sampler;
                                    param.samplingType = sampling;
                                    param.seed = seed;
                                    param.stepSize = stepSize;
                                    param.useConstraint = useConstraint;
                                    param.useObstacle = obstacle;
                                    m_paramsMA.push_back(param);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

ParamsMA ConfigurationMA::getParams() {
    if (m_index >= m_paramsMA.size())
        return ParamsMA();

    updatePropertyStats(m_index);
    return m_paramsMA[m_index++];
}

std::vector<ParamsMA> ConfigurationMA::getParamsList() {
    return m_paramsMA;
}

size_t ConfigurationMA::numParams() {
    return m_paramsMA.size();
}

void ConfigurationMA::updatePropertyStats(size_t index) {
    const auto params = m_paramsMA[index];
    m_propertyCollector->setProperties(params.robotType, params.useObstacle, params.useConstraint, params.optimize, params.stepSize,
                                       params.samplerType, params.samplingType, params.plannerType, params.pathModifier);
}

std::vector<std::string> ConfigurationMA::getSeeds() {
    return std::vector<std::string>({"234r5fdsfda", "23r54wedf", "23894rhwef", "092yu4re", "0923ujrpiofesd", "02u9r3jes",
                                     "09243rjpef", "23refdss;wf", "2-3r0wepoj", "-243refdsaf"});
}

std::vector<double> ConfigurationMA::getMobileStepSizes() {
    return std::vector<double>({25, 40, 55});
}

std::vector<double> ConfigurationMA::getSerialStepSizes() {
    std::vector<double> rads = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55};
    return util::toRad(rads);
}

} /* namespace ippp */
