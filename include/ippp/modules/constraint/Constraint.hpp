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

#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

#include <ippp/Identifier.h>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base class of all constraint checker and projections.
* \author  Sascha Kaden
* \date    2018-01-08
*/
template <unsigned int dim>
class Constraint : public Identifier {
  public:
    Constraint(const std::string &name, const std::shared_ptr<Environment> &environment);

    virtual bool checkConfig(const Vector<dim> &config) = 0;
    virtual Vector<dim> projectConfig(const Vector<dim> &config) = 0;

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;
};

/*!
*  \brief      Constructor of the class Constraint
*  \param[in]  name
*  \param[in]  Environment
*  \author     Sascha Kaden
*  \date       2018-01-08
*/
template <unsigned int dim>
Constraint<dim>::Constraint(const std::string &name, const std::shared_ptr<Environment> &environment)
    : Identifier(name), m_environment(environment) {
}

} /* namespace ippp */

#endif /* CONSTRAINT_HPP */