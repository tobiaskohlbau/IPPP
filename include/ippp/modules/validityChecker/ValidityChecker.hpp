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

#ifndef VALIDITYCHECKER_HPP
#define VALIDITYCHECKER_HPP

#include <vector>

#include <ippp/Identifier.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/modules/constraint/Constraint.hpp>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base class of all ValidityChecker.
* \author  Sascha Kaden
* \date    2018-01-17
*/
template <unsigned int dim>
class ValidityChecker : public Identifier {
  public:
    ValidityChecker(const std::shared_ptr<Environment> &environment, const std::shared_ptr<CollisionDetection<dim>> &collision,
                    const std::shared_ptr<Constraint<dim>> &constraint = nullptr, const std::string &name = "ValidityChecker");

    virtual bool checkConfig(const Vector<dim> &config) const;
    virtual bool checkTrajectory(const std::vector<Vector<dim>> &config) const;

  protected:
    bool checkRobotBound(const Vector<dim> &config) const;
    bool checkRobotBound(const std::vector<Vector<dim>> &config) const;

    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Constraint<dim>> m_constraint = nullptr;
    std::shared_ptr<Environment> m_environment;

    std::pair<Vector<dim>, Vector<dim>> m_robotBounding; /*!< Boundaries of the robot, fetched from the Environment */
};

/*!
*  \brief      Constructor of the class ValidityChecker
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  Constraint
*  \param[in]  name
*  \author     Sascha Kaden
*  \date       2018-01-17
*/
template <unsigned int dim>
ValidityChecker<dim>::ValidityChecker(const std::shared_ptr<Environment> &environment,
                                      const std::shared_ptr<CollisionDetection<dim>> &collision,
                                      const std::shared_ptr<Constraint<dim>> &constraint, const std::string &name)
    : Identifier(name),
      m_collision(collision),
      m_constraint(constraint),
      m_environment(environment),
      m_robotBounding(environment->getRobotBoundaries()) {
    Logging::debug("Initialize", this);
}

/*!
*  \brief      Checks the validity of the passed configuration, return true if valid.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity, true if valid
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ValidityChecker<dim>::checkConfig(const Vector<dim> &config) const {
    if (!checkRobotBound(config))
        return false;

    if (m_constraint) {
        if (!m_collision->checkConfig(config) && m_constraint->checkConfig(config))
            return true;

        return false;
    }

    return !m_collision->checkConfig(config);
}

/*!
*  \brief      Checks the validity of the passed configurations, return true if valid.
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] validity, true if valid
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ValidityChecker<dim>::checkTrajectory(const std::vector<Vector<dim>> &configs) const {
    if (!checkRobotBound(configs))
        return false;

    if (m_constraint) {
        if (!m_collision->checkTrajectory(configs) && m_constraint->checkTrajectory(configs))
            return true;

        return false;
    }

    return !m_collision->checkTrajectory(configs);
}

/*!
*  \brief      Checks the boundaries of the robot from the passed configuration, return true if valid.
*  \detail     The boundaries of the robot are taken from environment.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity of boundary check.
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ValidityChecker<dim>::checkRobotBound(const Vector<dim> &config) const {
    for (unsigned int i = 0; i < dim; ++i) {
        if (config[i] < m_robotBounding.first[i] || m_robotBounding.second[i] < config[i]) {
            Logging::trace("Robot out of robot boundary", this);
            return false;
        }
    }
    return true;
}

/*!
*  \brief      Checks the boundaries of the robot from the passed configurations, return true if valid.
*  \detail     The boundaries of the robot are taken from environment.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity of boundary check.
*  \date       2018-01-17
*/
template <unsigned int dim>
bool ValidityChecker<dim>::checkRobotBound(const std::vector<Vector<dim>> &configs) const {
    for (const auto &config : configs)
        if (!checkRobotBound(config))
            return false;
    return true;
}

} /* namespace ippp */

#endif /* CONSTRAINT_HPP */