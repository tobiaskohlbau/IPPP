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

#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include <ippp/modules/validityChecker/ValidityChecker.hpp>

namespace ippp {

/*!
* \brief   Base class of all constraint checker.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class Constraint : public ValidityChecker<dim> {
  public:
    Constraint(const std::string &name, const std::shared_ptr<Environment> &environment, double epsilon = IPPP_EPSILON);

    virtual Vector6 calcEuclideanError(const Vector<dim> &config) const = 0;
};

/*!
*  \brief      Constructor of the base class of all constraint modules
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  epsilon
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
Constraint<dim>::Constraint(const std::string &name, const std::shared_ptr<Environment> &environment, double epsilon)
    : ValidityChecker<dim>(name, environment, epsilon) {
}

} /* namespace ippp */

#endif /* CONSTRAINT_HPP */