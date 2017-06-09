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

#include <core/Identifier.h>
#include <core/collisionDetection/CollisionDetection.hpp>
#include <core/trajectoryPlanner/TrajectoryPlanner.hpp>
#include <core/types.h>
#include <environment/Environment.h>

namespace ippp {

/*!
* \brief   Base interface of all path modifier or path smoother.
* \author  Sascha Kaden
* \date    2017-05-23
*/
template <unsigned int dim>
class PathModifier : public Identifier {
  public:
    PathModifier(const std::string &name, const std::shared_ptr<Environment> &environment, std::shared_ptr<CollisionDetection<dim>> &collision,
                 std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    virtual std::vector<std::shared_ptr<Node<dim>>> smoothPath(const std::vector<std::shared_ptr<Node<dim>>> &nodes) const = 0;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Constructor of the PathModifier interface.
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \date       2017-05-23
*/
template <unsigned int dim>
PathModifier<dim>::PathModifier(const std::string &name, const std::shared_ptr<Environment> &environment,
                                std::shared_ptr<CollisionDetection<dim>> &collision,
                                std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : Identifier(name), m_collision(collision), m_environment(environment), m_trajectory(trajectory) {
}

} /* namespace ippp */

#endif /* PATHMODIFIER_HPP */
