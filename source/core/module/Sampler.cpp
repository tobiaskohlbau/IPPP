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

#include <core/module/Sampler.h>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>

namespace rmpl {

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \param[in]  SamplingStrategy
*  \date       2016-05-24
*/
Sampler::Sampler(const std::shared_ptr<RobotBase> &robot, SamplingMethod method) : ModuleBase("Sampler"), m_dim(robot->getDim()) {
    m_method = method;

    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();
    if (utilVec::empty(m_minBoundary) || utilVec::empty(m_maxBoundary))
        Logging::error("Boundaries are empty", this);

    m_generator = std::mt19937(rd());

    for (unsigned int i = 0; i < m_dim; ++i) {
        std::normal_distribution<float> dist1(0, 500);
        m_distNormal.push_back(dist1);

        std::uniform_real_distribution<float> dist2(m_minBoundary[i], m_maxBoundary[i]);
        m_distUniform.push_back(dist2);
    }
    m_distAngle = std::uniform_real_distribution<float>(0, utilGeo::twoPi());
}

/*!
*  \brief      Return sample
*  \author     Sascha Kaden
*  \param[out] sample Vec
*  \date       2016-05-24
*/
Eigen::VectorXf Sampler::getSample() {
    switch (m_method) {
        case SamplingMethod::standardDistribution:
            return sampleStandardDist();
        case SamplingMethod::uniform:
            return sampleUniform();
        default:
            return sampleRandom();
    }
}

/*!
*  \brief      Return random angle in rad
*  \author     Sascha Kaden
*  \param[out] rad angle
*  \date       2016-12-20
*/
float Sampler::getRandomAngle() {
    return m_distAngle(m_generator);
}

/*!
*  \brief      Set the mean of the standard distribution
*  \author     Sascha Kaden
*  \param[in]  mean of distribution
*  \param[out] binary result
*  \date       2016-11-14
*/
bool Sampler::setMeanOfDistribution(const Eigen::VectorXf &mean) {
    if (mean.rows() != m_dim) {
        Logging::error("Wrong dimension of mean vector", this);
        return false;
    }

    m_distNormal.clear();
    for (unsigned int i = 0; i < m_dim; ++i) {
        std::normal_distribution<float> distribution(mean[i], m_maxBoundary[i] - m_minBoundary[i]);
        m_distNormal.push_back(distribution);
    }
    return true;
}

/*!
*  \brief      Create a sample by the standard distribution and the set mean value
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[out] sample Vec
*  \date       2016-11-14
*/
Eigen::VectorXf Sampler::sampleStandardDist() {
    Eigen::VectorXf vec = utilVec::Vecf(m_dim);
    float number;
    for (unsigned int i = 0; i < m_dim; ++i) {
        do {
            number = m_distNormal[i](m_generator);
        } while ((number <= m_minBoundary[i]) || (number >= m_maxBoundary[i]));
        vec[i] = number;
    }
    return vec;
}

/*!
*  \brief      Create a sample by a uniform distribution
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[out] sample Vec
*  \date       2016-11-14
*/
Eigen::VectorXf Sampler::sampleUniform() {
    Eigen::VectorXf vec = utilVec::Vecf(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        vec[i] = m_distUniform[i](m_generator);
    return vec;
}

/*!
*  \brief      Create a random sample
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[out] sample Vec
*  \date       2016-11-14
*/
Eigen::VectorXf Sampler::sampleRandom() {
    Eigen::VectorXf vec = utilVec::Vecf(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        vec[i] = m_minBoundary[i] + (float)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
    return vec;
}

} /* namespace rmpl */