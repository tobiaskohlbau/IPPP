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

#ifndef SAMPLER_H_
#define SAMPLER_H_

#include <math.h>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <core/module/ModuleBase.h>
#include <robot/RobotBase.h>

namespace rmpl {

enum SamplingMethod { randomly, uniform, standardDistribution };

/*!
* \brief   Class Sampling creates sample vecs with the configurated method
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Sampler : public ModuleBase {
  public:
    Sampler(const std::shared_ptr<RobotBase> &robot, SamplingMethod method = SamplingMethod::randomly);
    Eigen::VectorXf getSample();
    float getRandomAngle();

    bool setMeanOfDistribution(const Eigen::VectorXf &mean);

  private:
    Eigen::VectorXf sampleStandardDist();
    Eigen::VectorXf sampleUniform();
    Eigen::VectorXf sampleRandom();

    Eigen::VectorXf m_minBoundary;
    Eigen::VectorXf m_maxBoundary;
    SamplingMethod m_method;

    std::random_device rd;
    std::mt19937 m_generator;
    std::vector<std::normal_distribution<float>> m_distNormal;
    std::vector<std::uniform_real_distribution<float>> m_distUniform;
    std::uniform_real_distribution<float> m_distAngle;
};

/*!
*  \brief      Constructor of the class Sampling
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  SamplingMethod
*  \param[in]  SamplingStrategy
*  \date       2016-05-24
*/
    template <unsigned int dim>
Sampler<dim>::Sampler(const std::shared_ptr<RobotBase> &robot, SamplingMethod method) : ModuleBase("Sampler") {
    m_method = method;

    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();
    if (utilVec::empty(m_minBoundary) || utilVec::empty(m_maxBoundary))
        Logging::error("Boundaries are empty", this);

    m_generator = std::mt19937(rd());

    for (unsigned int i = 0; i < dim; ++i) {
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
    template <unsigned int dim>
Eigen::VectorXf Sampler<dim>::getSample() {
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
    template <unsigned int dim>
float Sampler<dim>::getRandomAngle() {
    return m_distAngle(m_generator);
}

/*!
*  \brief      Set the mean of the standard distribution
*  \author     Sascha Kaden
*  \param[in]  mean of distribution
*  \param[out] binary result
*  \date       2016-11-14
*/
template <unsigned int dim>
bool Sampler<dim>::setMeanOfDistribution(const Eigen::VectorXf &mean) {
    if (mean.rows() != dim) {
        Logging::error("Wrong dimension of mean vector", this);
        return false;
    }

    m_distNormal.clear();
    for (unsigned int i = 0; i < dim; ++i) {
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
    template <unsigned int dim>
Eigen::VectorXf Sampler<dim>::sampleStandardDist() {
    Eigen::VectorXf vec = utilVec::Vecf(dim);
    float number;
    for (unsigned int i = 0; i < dim; ++i) {
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
    template <unsigned int dim>
Eigen::VectorXf Sampler<dim>::sampleUniform() {
    Eigen::VectorXf vec = utilVec::Vecf(dim);
    for (unsigned int i = 0; i < dim; ++i)
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
    template <unsigned int dim>
Eigen::VectorXf Sampler<dim>::sampleRandom() {
    Eigen::VectorXf vec = utilVec::Vecf(dim);
    for (unsigned int i = 0; i < dim; ++i)
        vec[i] = m_minBoundary[i] + (float)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
    return vec;
}

} /* namespace rmpl */

#endif /* SAMPLER_H_ */
