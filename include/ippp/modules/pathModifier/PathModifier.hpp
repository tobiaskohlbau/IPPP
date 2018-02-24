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

#ifndef PATHMODIFIER_HPP
#define PATHMODIFIER_HPP

#include <ippp/Identifier.h>
#include <ippp/environment/Environment.h>
#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>
#include <ippp/modules/validityChecker/ValidityChecker.hpp>
#include <ippp/types.h>

namespace ippp {

/*!
* \brief   Base interface of all path modifier or path smoother.
* \author  Sascha Kaden
* \date    2017-05-23
*/
template <unsigned int dim>
class PathModifier : public Identifier {
  public:
    PathModifier(const std::string &name, const std::shared_ptr<Environment> &environment,
                 const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                 const std::shared_ptr<ValidityChecker<dim>> &validityChecker);

    virtual std::vector<std::shared_ptr<Node<dim>>> smoothPath(const std::vector<std::shared_ptr<Node<dim>>> &nodes) const = 0;

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;              /*!< pointer to the Environment */
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;    /*!< pointer to the trajectory planning module */
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr; /*!< pointer to the ValidityChecker module */
};

/*!
*  \brief      Constructor of the PathModifier interface.
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  TrajectoryPlanner
*  \param[in]  ValidityChecker
*  \date       2017-05-23
*/
template <unsigned int dim>
PathModifier<dim>::PathModifier(const std::string &name, const std::shared_ptr<Environment> &environment,

                                const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker)
    : Identifier(name), m_environment(environment), m_validityChecker(validityChecker), m_trajectory(trajectory) {
    Logging::debug("Initialize", this);
}

} /* namespace ippp */

#endif /* PATHMODIFIER_HPP */
