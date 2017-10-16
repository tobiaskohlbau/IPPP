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

#ifndef ENVIRONMENTCONFIGURATOR_H
#define ENVIRONMENTCONFIGURATOR_H

#include <type_traits>
#include <vector>

#include <ui/Configurator.h>

#include <ippp/Environment.h>

namespace ippp {

/*!
* \brief   Class EnvironmentConfigurator constructs the environment of the planner
* \author  Sascha Kaden
* \date    2017-10-16
*/
template <unsigned int dim>
class EnvironmentConfigurator : public Configurator {
  public:
    EnvironmentConfigurator();

  private:
};

/*!
*  \brief      Constructor of the class EnvironmentConfigurator
*  \author     Sascha Kaden
*  \date       2017-10-16
*/
template <unsigned int dim>
EnvironmentConfigurator<dim>::EnvironmentConfigurator() : Configurator("EnvironmentConfigurator") {
}

} /* namespace ippp */

#endif    // ENVIRONMENTCONFIGURATOR_H
