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

#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <math.h>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <ippp/Identifier.h>
#include <ippp/environment/Environment.h>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base class of all Sampler modules, return random samples.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Sampler : public Identifier {
  public:
    Sampler(const std::string &name, const std::shared_ptr<Environment> &environment, const std::string &seed = "");
    Sampler(const std::string &name, const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary,
            const std::string &seed = "");
    virtual Vector<dim> getSample() = 0;
    double getRandomAngle();
    double getRandomNumber();
    Vector<dim> getRandomRay();

    virtual void setOrigin(const Vector<dim> &mean);
    Vector<dim> getOrigin() const;

  protected:
    Vector<dim> m_minBoundary;
    Vector<dim> m_maxBoundary;
    Vector<dim> m_origin;

    std::random_device rd;
    std::minstd_rand0 m_generator;
    std::uniform_real_distribution<double> m_distAngle;
    std::uniform_real_distribution<double> m_distNumber;
};

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2016-05-24
*/
template <unsigned int dim>
Sampler<dim>::Sampler(const std::string &name, const std::shared_ptr<Environment> &environment, const std::string &seed)
    : Identifier(name) {
    Logging::debug("Initialize", this);

    auto boundaries = environment->getRobotBoundaries();
    m_minBoundary = boundaries.first;
    m_maxBoundary = boundaries.second;
    for (unsigned int i = 0; i < dim; ++i)
        assert(m_minBoundary[i] != m_maxBoundary[i]);

    m_origin = Vector<dim>::Zero();

    if (seed.empty()) {
        m_generator = std::minstd_rand0(rd());
    } else {
        std::seed_seq seed_seq(seed.begin(), seed.end());
        m_generator = std::minstd_rand0(seed_seq);
    }

    m_distAngle = std::uniform_real_distribution<double>(0, util::twoPi());
    m_distNumber = std::uniform_real_distribution<double>(0, 1.0);
}

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \date       2016-05-24
*/
template <unsigned int dim>
Sampler<dim>::Sampler(const std::string &name, const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary,
                      const std::string &seed)
    : Identifier(name) {
    Logging::debug("Initialize", this);

    m_minBoundary = minBoundary;
    m_maxBoundary = maxBoundary;
    for (unsigned int i = 0; i < dim; ++i)
        assert(m_minBoundary[i] != m_maxBoundary[i]);

    if (seed.empty()) {
        m_generator = std::minstd_rand0(rd());
    } else {
        std::seed_seq seed_seq(seed.begin(), seed.end());
        m_generator = std::minstd_rand0(seed_seq);
    }

    m_distAngle = std::uniform_real_distribution<double>(0, util::twoPi());
    m_distNumber = std::uniform_real_distribution<double>(0, 1.0);
}

/*!
*  \brief      Return random angle in rad
*  \author     Sascha Kaden
*  \param[out] rad angle
*  \date       2016-12-20
*/
template <unsigned int dim>
double Sampler<dim>::getRandomAngle() {
    return m_distAngle(m_generator);
}

/*!
*  \brief      Return random number between 0 and 1
*  \author     Sascha Kaden
*  \param[out] random number
*  \date       2017-04-03
*/
template <unsigned int dim>
double Sampler<dim>::getRandomNumber() {
    return m_distNumber(m_generator);
}

/*!
*  \brief      Return random normalized ray.
*  \author     Sascha Kaden
*  \param[out] random ray
*  \date       2017-04-03
*/
template <unsigned int dim>
Vector<dim> Sampler<dim>::getRandomRay() {
    Vector<dim> ray;
    for (unsigned int i = 0; i < dim; ++i)
        ray[i] = m_generator();
    return ray.normalized();
}

/*!
*  \brief      Set the origin of the Sampler
*  \author     Sascha Kaden
*  \param[in]  origin
*  \date       2016-11-14
*/
template <unsigned int dim>
void Sampler<dim>::setOrigin(const Vector<dim> &origin) {
    m_origin = origin;
}

/*!
*  \brief      Return the origin of the Sampler
*  \author     Sascha Kaden
*  \param[out] origin
*  \date       2016-11-14
*/
template <unsigned int dim>
Vector<dim> Sampler<dim>::getOrigin() const {
    return m_origin;
}

} /* namespace ippp */

#endif /* SAMPLER_HPP */
