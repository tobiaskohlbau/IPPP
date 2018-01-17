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

#ifndef EUCLIDEANCONSTRAINT_HPP
#define EUCLIDEANCONSTRAINT_HPP

#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   EuclideanConstraint transforms the configuration to the TCP and checks it.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class EuclideanConstraint : public Constraint<dim> {
  public:
    EuclideanConstraint(const std::shared_ptr<Environment> &environment, const Vector6 &constraint, double epsilon = EPSILON);

    bool checkConfig(const Vector<dim> &config);
    bool checkTrajectory(const std::vector<Vector<dim>> &config);
    double calcError(const Vector<dim> &config);
    Vector<dim> projectConfig(const Vector<dim> &config);

    void setConstraint(const Vector6 &constraint);
    Vector6 getConstraint() const;

  private:
    const double m_epsilon;
    Vector6 m_constraint;
    Eigen::Matrix<bool, 6, 1> m_checkConstraint;
    std::shared_ptr<RobotBase> m_robot = nullptr;
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
EuclideanConstraint<dim>::EuclideanConstraint(const std::shared_ptr<Environment> &environment, const Vector6 &constraint,
                                              double epsilon)
    : Constraint<dim>("EuclideanConstraint", environment), m_epsilon(epsilon) {
    setConstraint(constraint);
    m_robot = environment->getRobot();
}

/*!
*  \brief      Check the euclidean constraint of the configuration and return true if valid.
*  \param[in]  configuration
*  \param[out] result, true if valid
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
bool EuclideanConstraint<dim>::checkConfig(const Vector<dim> &config) {
    Vector6 tcpPose = util::transformToVec(m_robot->getTransformation(config));

    for (unsigned int i = 0; i < 6; ++i) {
        if (m_checkConstraint[i])
            if (std::abs(tcpPose[i] - m_constraint[i]) > m_epsilon)
                return false;
    }
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
bool EuclideanConstraint<dim>::checkTrajectory(const std::vector<Vector<dim>> &configs) {
    for (auto &config : configs)
        if (!checkConfig(config))
            return false;

    return true;
}

/*!
*  \brief      Dummy calculation of the configuration erroe, returns always 0.
*  \param[in]  configuration
*  \param[out] zero
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
double EuclideanConstraint<dim>::calcError(const Vector<dim> &config) {
    Vector6 tcpPose = util::transformToVec(m_robot->getTransformation(config));

    return (tcpPose - m_constraint).norm();
}

/*!
*  \brief      Project the passed config until it is valid.
*  \param[in]  configuration
*  \param[out] valid configuration in relation to the constraint
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
Vector<dim> EuclideanConstraint<dim>::projectConfig(const Vector<dim> &config) {
    return config;
}

/*!
*  \brief      Sets the euclidean constraint vector.
*  \param[in]  constraint vector
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
void EuclideanConstraint<dim>::setConstraint(const Vector6 &constraint) {
    m_constraint = constraint;
    // init flag vector for faster check
    for (size_t i = 0; i < 6; ++i) {
        if (std::isnan(constraint[i]))
            m_checkConstraint[i] = false;
        else
            m_checkConstraint[i] = true;
    }
}

/*!
*  \brief      Returns the euclidean constraint vector.
*  \param[out] constraint vector
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
Vector6 EuclideanConstraint<dim>::getConstraint() const {
    return m_constraint;
}

} /* namespace ippp */

#endif /* EUCLIDEANCONSTRAINT_HPP */