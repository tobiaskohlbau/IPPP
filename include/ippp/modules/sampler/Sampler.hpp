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

#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <random>
#include <string>

#include <ippp/Identifier.h>
#include <ippp/environment/Environment.h>
#include <ippp/types.h>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Base class of all Sampler modules, it returns random generated samples.
* \details The sampler is using the std::minstd_rand0 generator for the calculation of the samples. If no seed is passed, a random
* seed will be generated. The Sampler has to be used to generate random numbers to guarantee the determinableness of the
* MotionPlanner
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Sampler : public Identifier {
  public:
    Sampler(const std::string &name, const std::shared_ptr<Environment> &environment, const std::string &seed = "");
    Sampler(const std::string &name, const std::pair<Vector<dim>, Vector<dim>> &boundary, const std::string &seed = "");
    virtual Vector<dim> getSample() = 0;
    double getRandomAngle();
    double getRandomNumber();
    Vector<dim> getRandomRay();

    virtual void setOrigin(const Vector<dim> &mean);
    virtual void setOptimalPathCost(double cost);
    Vector<dim> getOrigin() const;

  protected:
    std::pair<Vector<dim>, Vector<dim>> m_robotBoundary; /*!< boundary of the robot (pair with min and max) */
    Vector<dim> m_origin;                                /*!< origin of the sampler (used for normal distribution) */
    double m_optimalPathCost = 1;

    std::random_device rd;                               /*!< random device, will be used, if no seed is passed */
    std::minstd_rand0 m_generator;                       /*!< generator for the different distributions */
    std::uniform_real_distribution<double> m_distAngle;  /*!< distribution to generate random rad angle [0,2*pi] */
    std::uniform_real_distribution<double> m_distNumber; /*!< distriburtion to generate a random number [0,1] */
};

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
Sampler<dim>::Sampler(const std::string &name, const std::shared_ptr<Environment> &environment, const std::string &seed)
    : Identifier(name), m_origin(Vector<dim>::Zero()), m_optimalPathCost(1) {
    // Logging::debug("Initialize", this);

    m_robotBoundary = environment->getRobotBoundaries();
    for (unsigned int i = 0; i < dim; ++i)
        assert(m_robotBoundary.first[i] != m_robotBoundary.second[i]);

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
*  \param[in]  name
*  \param[in]  robot boundary
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
Sampler<dim>::Sampler(const std::string &name, const std::pair<Vector<dim>, Vector<dim>> &boundary, const std::string &seed)
    : Identifier(name), m_origin(Vector<dim>::Zero()), m_optimalPathCost(1) {
    // Logging::debug("Initialize", this);

    m_robotBoundary = boundary;
    for (unsigned int i = 0; i < dim; ++i)
        assert(m_robotBoundary.first[i] != m_robotBoundary.second[i]);

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
*  \brief      Set optimal cost of path
*  \author     Sascha Kaden
*  \param[in]  optimal path cost
*  \date       2019-03-04
*/
template <unsigned int dim>
void Sampler<dim>::setOptimalPathCost(double cost) {
    if (cost <= 0) {
        Logging::error("Optimal cost has to be larger than 0", this);
        return;
    }
    m_optimalPathCost = cost;
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
