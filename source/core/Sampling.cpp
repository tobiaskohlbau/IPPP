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

#include <core/Sampling.h>

#include <core/Logging.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \date       2016-05-24
*/
Sampling::Sampling(const std::shared_ptr<RobotBase> &robot, SamplingMethod method) : ModuleBase("Sampling") {
    m_method = method;
    m_robot = robot;

    m_minBoundary = m_robot->getMinBoundary();
    m_maxBoundary = m_robot->getMaxBoundary();

    srand(time(NULL));

    m_generator = std::mt19937(rd());
    for (unsigned int i = 0; i < m_robot->getDim(); ++i) {
        std::normal_distribution<float> dist1(0, 500);
        m_distNormal.push_back(dist1);

        std::uniform_real_distribution<float> dist2(m_minBoundary[i], m_maxBoundary[i]);
        m_distUniform.push_back(dist2);
    }
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  index from loop of samples
*  \param[in]  absolute number of samples, which will be created
*  \param[out] Vec
*  \date       2016-05-24
*/
Vec<float> Sampling::getSample(unsigned int dim, int index, int nbSamples) {
    Vec<float> vec(dim);
    if (!checkBoudaries())
        return vec;

    if (m_method == SamplingMethod::standardDistribution) {
        float number;
        for (unsigned int i = 0; i < dim; ++i) {
            do {
                number = m_distNormal[i](m_generator);
            } while ((number <= m_minBoundary[i]) || (number >= m_maxBoundary[i]));
            vec[i] = number;
        }
        return vec;
    } else if (m_method == SamplingMethod::uniform) {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_distUniform[i](m_generator);
        return vec;
    } else {
        for (unsigned int i = 0; i < dim; ++i)
            vec[i] = m_minBoundary[i] + (float)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
        return vec;
    }
}

bool Sampling::setMeanOfDistribution(const Vec<float> &mean) {
    if (mean.getDim() != m_robot->getDim()) {
        Logging::warning("Wrong dimension of mean vector", this);
        return false;
    }

    m_distNormal.clear();
    for (unsigned int i = 0; i < m_robot->getDim(); ++i) {
        std::normal_distribution<float> distribution(mean[i], m_maxBoundary[i] - m_minBoundary[i]);
        m_distNormal.push_back(distribution);
    }
    return true;
}

/*!
*  \brief      Check boundary existence
*  \author     Sascha Kaden
*  \param[out] result of check
*  \date       2016-05-24
*/
bool Sampling::checkBoudaries() {
    if (m_minBoundary.empty() || m_maxBoundary.empty()) {
        Logging::warning("Boundaries are empty", this);
        return false;
    }
    return true;
}
