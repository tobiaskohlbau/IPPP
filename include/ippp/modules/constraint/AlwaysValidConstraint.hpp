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

#ifndef ALWAYSVALIDCONSTRAINT_HPP
#define ALWAYSVALIDCONSTRAINT_HPP

#include <ippp/modules/constraint/Constraint.hpp>

namespace ippp {

/*!
* \brief   AlwaysValidConstraint return always valid by checking of configurations.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class AlwaysValidConstraint : public Constraint<dim> {
  public:
    AlwaysValidConstraint(const std::shared_ptr<Environment> &environment);

    bool checkConfig(const Vector<dim> &config);
    Vector<dim> projectConfig(const Vector<dim> &config);
};

/*!
*  \brief      Constructor of the class AlwaysValidConstraint
*  \param[in]  Environment
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
AlwaysValidConstraint<dim>::AlwaysValidConstraint(const std::shared_ptr<Environment> &environment)
    : Constraint("EuclideanConstriant", environment) {
}

/*!
*  \brief      Dummy check config, returns always valid.
*  \param[in]  configuration
*  \param[out] true
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
bool AlwaysValidConstraint<dim>::checkConfig(const Vector<dim> &config) {
    return true;
}

/*!
*  \brief      Dummy check config, returns the passed configuration.
*  \param[in]  configuration
*  \param[out] passed configuration
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
Vector<dim> AlwaysValidConstraint<dim>::projectConfig(const Vector<dim> &config) {
    return config;
}

} /* namespace ippp */

#endif /* ALWAYSVALIDCONSTRAINT_HPP */