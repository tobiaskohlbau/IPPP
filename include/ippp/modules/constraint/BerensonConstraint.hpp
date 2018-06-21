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

#ifndef BERENSONCONSTRAINT_HPP
#define BERENSONCONSTRAINT_HPP

#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   EuclideanConstraint transforms the configuration to the TCP and checks it.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class BerensonConstraint : public Constraint<dim> {
  public:
    BerensonConstraint(const std::shared_ptr<Environment> &environment, const Transform &taskFrame,
                       const std::pair<Vector6, Vector6> &C, double epsilon = IPPP_EPSILON);

    bool check(const Vector<dim> &config) const;
    bool check(const std::vector<Vector<dim>> &config) const;
    double calc(const Vector<dim> &config) const override;
    double calc(const std::vector<Vector<dim>> &configs) const override;
    Vector6 calcEuclideanError(const Vector<dim> &config) const;

  private:
    std::pair<Vector6, Vector6> m_C;
    Transform m_taskFrame;
    Transform m_taskFrameInv;
    std::shared_ptr<SerialRobot> m_serialRobot = nullptr;

    using Constraint<dim>::m_epsilon;
};

/*!
*  \brief      Constructor of the class EuclideanConstraint
*  \param[in]  Environment
*  \param[in]  euclidean constraint vector (pose)
*  \param[in]  distance epsilon
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
BerensonConstraint<dim>::BerensonConstraint(const std::shared_ptr<Environment> &environment, const Transform &taskFrame,
                                            const std::pair<Vector6, Vector6> &C, double epsilon)
    : Constraint<dim>("BerensonConstraint", environment, epsilon),
      m_serialRobot(std::dynamic_pointer_cast<SerialRobot>(environment->getRobot())),
      m_taskFrame(taskFrame),
      m_taskFrameInv(taskFrame.inverse()),
      m_C(C) {
}

/*!
*  \brief      Check the euclidean constraint of the configuration and return true if valid.
*  \param[in]  configuration
*  \param[out] result, true if valid
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
bool BerensonConstraint<dim>::check(const Vector<dim> &config) const {
    if (calc(config) > 0)
        return false;
    return true;
}

template <unsigned int dim>
bool BerensonConstraint<dim>::check(const std::vector<Vector<dim>> &configs) const {
    if (calc(configs) > 0)
        return false;

    return true;
}

template <unsigned int dim>
double BerensonConstraint<dim>::calc(const Vector<dim> &config) const {
    return (calcEuclideanError(config).cwiseAbs()).maxCoeff() - m_epsilon;
}

template <unsigned int dim>
double BerensonConstraint<dim>::calc(const std::vector<Vector<dim>> &configs) const {
    double maxError = -std::numeric_limits<double>::max();
    double error;
    for (auto &config : configs) {
        error = calc(config);
        if (error > maxError)
            maxError = error;
    }
    return maxError;
}

template <unsigned int dim>
Vector6 BerensonConstraint<dim>::calcEuclideanError(const Vector<dim> &config) const {
    auto T = m_taskFrameInv * m_serialRobot->getTransformation(config);
    AngleAxis angleAxis(T.rotation());
    Vector6 eucError = util::append<3, 3>(T.translation(), angleAxis.axis() * angleAxis.angle());
    for (size_t i = 0; i < 6; ++i) {
        if (eucError[i] > m_C.second[i])
            eucError[i] -= m_C.second[i];
        else if (eucError[i] < m_C.first[i])
            eucError[i] -= m_C.first[i];
        else
            eucError[i] = 0;
    }
    //std::cout << eucError << std::endl;
    return eucError;
}

} /* namespace ippp */

#endif /* BERENSONCONSTRAINT_HPP */