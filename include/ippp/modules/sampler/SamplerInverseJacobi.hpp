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

#ifndef SAMPLERNINVERSEJACOBI_HPP
#define SAMPLERNINVERSEJACOBI_HPP

#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/sampler/Sampler.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Class SamplerNormalDist create samples by a normal distribution.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class SamplerInverseJacobi : public Sampler<dim> {
  public:
    SamplerInverseJacobi(const std::shared_ptr<Environment> &environment, const Vector6 &goalPose, const std::string &seed = "");
    Vector<dim> getSample() override;
    void setOrigin(const Vector<dim> &origin) override;

  private:
    double calcValue(unsigned int index);
    Vector<dim> incrementSample(Vector<dim> &start);

    std::vector<std::normal_distribution<double>> m_distNormal;
    Vector6 m_goalPose;
    Transform m_goalTransform;
    std::shared_ptr<SerialRobot> m_robot;

    using Sampler<dim>::m_generator;
    using Sampler<dim>::m_origin;
    using Sampler<dim>::m_robotBoundary;
};

/*!
*  \brief      Constructor of the class SamplerNormalDist
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  seed
*  \date       2016-05-24
*/
template <unsigned int dim>
SamplerInverseJacobi<dim>::SamplerInverseJacobi(const std::shared_ptr<Environment> &environment, const Vector6 &goalPose,
                                                const std::string &seed)
    : Sampler<dim>("SamplerInverseJacobi", environment, seed),
      m_goalPose(goalPose),
      m_goalTransform(util::toTransform(goalPose)),
      m_robot(std::dynamic_pointer_cast<SerialRobot>(environment->getRobot())) {
    setOrigin(m_origin);
}

/*!
*  \brief      Return sample by a normal distribution
*  \author     Sascha Kaden
*  \param[out] sample
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> SamplerInverseJacobi<dim>::getSample() {
    Vector<dim> config;
    for (unsigned int i = 0; i < dim; ++i)
        config[i] = calcValue(i);
    if (util::empty<dim>(config))
        return config;

    return incrementSample(config);
}

/*!
*  \brief      Set the origin of the normal distribution
*  \author     Sascha Kaden
*  \param[in]  origin
*  \date       2016-11-14
*/
template <unsigned int dim>
void SamplerInverseJacobi<dim>::setOrigin(const Vector<dim> &origin) {
    m_distNormal.clear();
    for (unsigned int i = 0; i < dim; ++i) {
        std::normal_distribution<double> distribution(origin[i], (m_robotBoundary.second[i] - m_robotBoundary.first[i]) / 8);
        m_distNormal.push_back(distribution);
    }
}

template <unsigned int dim>
double SamplerInverseJacobi<dim>::calcValue(unsigned int index) {
    double value;
    for (size_t i = 0; i < 20; ++i) {
        value = m_distNormal[index](m_generator);
        if (value >= m_robotBoundary.first[index] && value <= m_robotBoundary.second[index])
            return value;
    }
    return std::nanf("1");
}

template <unsigned int dim>
Vector<dim> SamplerInverseJacobi<dim>::incrementSample(Vector<dim> &start) {
    auto goalRotation = m_goalTransform.rotation();
    Vector3 goalTranslation = m_goalTransform.translation();
    auto goalInverse = m_goalTransform.inverse();

    Vector6 diff;
    for (size_t i = 0; i < 150000; ++i) {
        auto startTransform = m_robot->getTransformation(start);
        AngleAxis angleAxis(startTransform.rotation().transpose() * goalRotation);
        diff = util::append<3, 3>(goalTranslation - startTransform.translation(), angleAxis.axis() * angleAxis.angle());

        if (diff.squaredNorm() < 0.001)
            break;
//        if (i == 199999) {
//            for (size_t j = 0; j < 6; ++j)
//                std::cout << diff[j] << " ";
//            std::cout << std::endl;
//        }

        diff.block<3,1>(3,0) *= 10;
        while (diff.squaredNorm() > 10)
            diff /= 10;
        while (diff.squaredNorm() > 2)
            diff /= 2;

        MatrixX invJ = m_robot->calcJacobian(start).completeOrthogonalDecomposition().pseudoInverse();
        start += (invJ * diff) * 0.5;
    }
    // reduce multiplies of pi
    for (size_t j = 0; j < dim; ++j) {
        while (start[j] > util::pi())
            start[j] -= util::twoPi();
        while (start[j] < -util::pi())
            start[j] += util::twoPi();
    }
    // check the boundary of the robot and set them
    for (size_t j = 0; j < dim; ++j) {
        if (start[j] > m_robotBoundary.second[j])
            start[j] = m_robotBoundary.second[j] - IPPP_EPSILON;
        if (start[j] < m_robotBoundary.first[j])
            start[j] = m_robotBoundary.first[j] + IPPP_EPSILON;
    }
    return start;
}

} /* namespace ippp */

#endif /* SAMPLERNINVERSEJACOBI_HPP */
