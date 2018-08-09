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

#ifndef ALWAYSTRUEVALIDITY_HPP
#define ALWAYSTRUEVALIDITY_HPP

#include <ippp/modules/validityChecker/ValidityChecker.hpp>

namespace ippp {

/*!
* \brief   Base class of all ValidityChecker.
* \author  Sascha Kaden
* \date    2018-01-17
*/
template <unsigned int dim>
class AlwaysTrueValidity : public ValidityChecker<dim> {
  public:
    AlwaysTrueValidity(const std::shared_ptr<Environment> &environment);

    bool check(const Vector<dim> &config) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  protected:
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
AlwaysTrueValidity<dim>::AlwaysTrueValidity(const std::shared_ptr<Environment> &environment)
    : ValidityChecker<dim>("AlwaysTrueValidity", environment) {
}

/*!
*  \brief      Calculates the validity of the passed configuration, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
bool AlwaysTrueValidity<dim>::check(const Vector<dim> &config) const {
    return true;
}

/*!
*  \brief      Calculates the validity of the passed configurations, return double validity value.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] validity
*  \date       2018-01-17
*/
template <unsigned int dim>
bool AlwaysTrueValidity<dim>::check(const std::vector<Vector<dim>> &configs) const {
    return true;
}

} /* namespace ippp */

#endif /* ALWAYSTRUEVALIDITY_HPP */