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

#include <core/module/Sampling.h>

#include <include/core/utility/Logging.h>

namespace rmpl {

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \param[in]  SamplingStrategy
*  \date       2016-05-24
*/
Sampling::Sampling(const std::shared_ptr<RobotBase> &robot, SamplingMethod method, SamplingStrategy strategy)
        : ModuleBase("Sampler") {
    m_strategy = strategy;
    m_robot = robot;
    m_sampler = std::shared_ptr<Sampler>(new Sampler(robot, method));
}

Vec<float> Sampling::getSample() {
    return m_sampler->getSample();
}

bool Sampling::setMeanOfDistribution(const Vec<float> &mean) {
    return m_sampler->setMeanOfDistribution(mean);
}

} /* namespace rmpl */