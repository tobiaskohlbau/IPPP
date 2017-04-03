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

#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <math.h>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <core/module/Identifier.h>
#include <core/types.h>
#include <robot/RobotBase.hpp>

namespace rmpl {

/*!
* \brief   Base class of all Sampler modules, return random samples.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Sampler : public Identifier {
  public:
    Sampler(const std::shared_ptr<RobotBase<dim>> &robot);
    virtual Vector<dim> getSample();
    float getRandomAngle();
    float getRandomNumber();

    virtual void setOrigin(const Vector<dim> &mean);
    Vector<dim> getOrigin() const;

  protected:
    Vector<dim> m_minBoundary;
    Vector<dim> m_maxBoundary;
    Vector<dim> m_origin;

    std::random_device rd;
    std::mt19937 m_generator;
    std::uniform_real_distribution<float> m_distAngle;
    std::uniform_real_distribution<float> m_distNumber;
};

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2016-05-24
*/
template <unsigned int dim>
Sampler<dim>::Sampler(const std::shared_ptr<RobotBase<dim>> &robot) : Identifier("Sampler") {
    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();
    Vector6 pose = robot->getPose();
    if (dim <= 6) {
        for (int i = 0; i < dim; ++i) {
            m_origin[i] = pose[i];
        }
    } else {
        for (int i = 0; i < 6; ++i) {
            m_origin[i] = pose[i];
        }
    }

    m_generator = std::mt19937(rd());
    m_distAngle = std::uniform_real_distribution<float>(0, util::twoPi());
    m_distNumber = std::uniform_real_distribution<float>(0, 1.0);
}

/*!
*  \brief      Return random sample
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> Sampler<dim>::getSample() {
    Vector<dim> vec;
    for (unsigned int i = 0; i < dim; ++i) {
        vec[i] = m_minBoundary[i] + (float)(m_generator() % (int)(m_maxBoundary[i] - m_minBoundary[i]));
    }
    return vec;
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
*  \brief      Return random number between 0 and 1
*  \author     Sascha Kaden
*  \param[out] random number
*  \date       2017-04-03
*/
template <unsigned int dim>
float Sampler<dim>::getRandomNumber() {
    return m_distNumber(m_generator);
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

} /* namespace rmpl */

#endif /* SAMPLER_HPP */
