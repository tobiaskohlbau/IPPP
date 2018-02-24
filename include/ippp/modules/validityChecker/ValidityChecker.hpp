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
#include <ippp/environment/Environment.h>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base class of all ValidityChecker.
* \author  Sascha Kaden
* \date    2018-01-17
*/
template <unsigned int dim>
class ValidityChecker : public Identifier {
  protected:
    ValidityChecker(const std::string &name, const std::shared_ptr<Environment> &environment, double epsilon = IPPP_EPSILON);

  public:
    virtual ~ValidityChecker();

    virtual bool check(const Vector<dim> &config) const = 0;
    virtual bool check(const std::vector<Vector<dim>> &configs) const = 0;
    virtual double calc(const Vector<dim> &config) const;
    virtual double calc(const std::vector<Vector<dim>> &config) const;

    bool checkRobotBound(const Vector<dim> &config) const;
    bool checkRobotBound(const std::vector<Vector<dim>> &config) const;
    void setEpsilon(double epsilon);
    double getEpsilon() const;

  protected:
    double m_epsilon = 0.1;
    std::shared_ptr<Environment> m_environment;
    std::pair<Vector<dim>, Vector<dim>> m_robotBounding; /*!< Boundaries of the robot, fetched from the Environment */
};

/*!
*  \brief      Constructor of the class ValidityChecker
*  \param[in]  name
*  \param[in]  Environment
*  \author     Sascha Kaden
*  \date       2018-01-17
*/
template <unsigned int dim>
ValidityChecker<dim>::ValidityChecker(const std::string &name, const std::shared_ptr<Environment> &environment, double epsilon)
    : Identifier(name), m_environment(environment), m_robotBounding(environment->getRobotBoundaries()) {
    Logging::debug("Initialize", this);
    setEpsilon(epsilon);
}

template <unsigned int dim>
ValidityChecker<dim>::~ValidityChecker() = default;

/*!
*  \brief      Calculates the validity of the passed configuration, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
double ValidityChecker<dim>::calc(const Vector<dim> &config) const {
    return static_cast<double>(check(config));
}

/*!
*  \brief      Calculates the validity of the passed configurations, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
double ValidityChecker<dim>::calc(const std::vector<Vector<dim>> &configs) const {
    return static_cast<double>(check(configs));
}

/*!
*  \brief      Set the epsilon value for checking of the configurations.
*  \author     Sascha Kaden
*  \param[in]  epsilon value
*  \date       2018-02-14
*/
template <unsigned int dim>
void ValidityChecker<dim>::setEpsilon(double epsilon) {
    if (epsilon < 0) {
        Logging::warning("Epsilon has to be larger than 0", this);
        return;
    }
    m_epsilon = epsilon;
}

/*!
*  \brief      Return the epsilon value for checking of the configurations.
*  \author     Sascha Kaden
*  \param[out] epsilon value
*  \date       2018-03-14
*/
template <unsigned int dim>
double ValidityChecker<dim>::getEpsilon() const {
    return m_epsilon;
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

#endif /* VALIDITYCHECKER_HPP */