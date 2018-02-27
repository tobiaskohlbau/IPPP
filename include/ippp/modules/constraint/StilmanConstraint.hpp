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

#ifndef STILMANCONSTRAINT_HPP
#define STILMANCONSTRAINT_HPP

#include <limits>

#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   EuclideanConstraint transforms the configuration to the TCP and checks it.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class StilmanConstraint : public Constraint<dim> {
  public:
    StilmanConstraint(const std::shared_ptr<Environment> &environment, const Transform &taskFrame,
                      const std::pair<Vector6, Vector6> &C, double epsilon = IPPP_EPSILON);

    bool check(const Vector<dim> &config) const;
    bool check(const std::vector<Vector<dim>> &config) const;
    double calc(const Vector<dim> &config) const override;
    double calc(const std::vector<Vector<dim>> &configs) const override;
    Vector6 calcEuclideanError(const Vector<dim> &config) const;

  private:
    Transform m_taskFrame;
    Transform m_taskFrameInv;
    std::pair<Vector6, Vector6> m_C;
    Matrix6 m_matC;
    std::shared_ptr<SerialRobot> m_serialRobot = nullptr;

    using ValidityChecker<dim>::m_epsilon;
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
StilmanConstraint<dim>::StilmanConstraint(const std::shared_ptr<Environment> &environment, const Transform &taskFrame,
                                          const std::pair<Vector6, Vector6> &C, double epsilon)
    : Constraint<dim>("StilmanConstraint", environment, epsilon),
      m_serialRobot(std::dynamic_pointer_cast<SerialRobot>(environment->getRobot())),
      m_taskFrame(taskFrame),
      m_taskFrameInv(taskFrame.inverse()),
      m_C(C) {
    m_matC = Matrix6::Zero();
    for (unsigned int i = 0; i < 6; ++i) {
        if (m_C.first[i] == -IPPP_MAX && m_C.second[i] == IPPP_MAX)
            continue;

        m_matC(i, i) = 1;
    }
}

/*!
*  \brief      Check the euclidean constraint of the configuration and return true if valid.
*  \param[in]  configuration
*  \param[out] result, true if valid
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
bool StilmanConstraint<dim>::check(const Vector<dim> &config) const {
    if (calc(config) > 0)
        return false;

    return true;
}

/*!
*  \brief      Check the euclidean constraint of the vector of configurations and return true if valid.
*  \param[in]  vector of configurations
*  \param[out] result, true if valid
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
bool StilmanConstraint<dim>::check(const std::vector<Vector<dim>> &configs) const {
    if (calc(configs) > 0)
        return false;

    return true;
}

template <unsigned int dim>
double StilmanConstraint<dim>::calc(const Vector<dim> &config) const {
    Vector6 eucError = calcEuclideanError(config);
    for (size_t i = 0; i < 6; ++i) {
        if (eucError[i] > m_C.second[i])
            eucError[i] -= m_C.second[i];
        else if (eucError[i] < m_C.first[i])
            eucError[i] -= m_C.first[i];
        else
            eucError[i] = 0;
    }
    return eucError.cwiseAbs().maxCoeff() - m_epsilon;
}

template <unsigned int dim>
double StilmanConstraint<dim>::calc(const std::vector<Vector<dim>> &configs) const {
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
Vector6 StilmanConstraint<dim>::calcEuclideanError(const Vector<dim> &config) const {
    auto T = m_taskFrameInv * m_serialRobot->getTransformation(config);
    Eigen::AngleAxisd angleAxis(T.rotation());
    //std::cout << "axis angle:" << std::endl;
    //std::cout << angleAxis.axis() << std::endl;
    //std::cout << "angle: " << angleAxis.angle() << std::endl;
    //Vector3 vec(T.translation());
    //Vector3 euler(T.rotation().eulerAngles(0, 1, 2));
    return m_matC * util::append<3, 3>(T.translation(), angleAxis.axis() * angleAxis.angle());

    // err x = C * delta x
    //return m_matC * util::transformToVec(m_taskFrameInv * m_serialRobot->getTransformation(config));
}

} /* namespace ippp */

#endif /* STILMANCONSTRAINT_HPP */