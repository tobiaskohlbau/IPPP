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

#ifndef DUMMYPATHMODIFIER_HPP
#define DUMMYPATHMODIFIER_HPP

#include <core/pathModifier/PathModifier.hpp>

namespace ippp {

/*!
* \brief   DummyPathModifier is dummy PathModifier, which only returns the passed path.
* \author  Sascha Kaden
* \date    2017-05-23
*/
template <unsigned int dim>
class DummyPathModifier : public PathModifier<dim> {
  public:
    DummyPathModifier(const std::shared_ptr<Environment> &environment, std::shared_ptr<CollisionDetection<dim>> &collision,
                      std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    std::vector<std::shared_ptr<Node<dim>>> smoothPath(const std::vector<std::shared_ptr<Node<dim>>> &nodes) const;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Constructor of the DummyPathModifier.
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \date       2017-05-23
*/
template <unsigned int dim>
DummyPathModifier<dim>::DummyPathModifier(const std::shared_ptr<Environment> &environment,
                                          std::shared_ptr<CollisionDetection<dim>> &collision,
                                          std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : PathModifier<dim>("Dummy PathModifier", environment, collision, trajectory) {
}

/*!
*  \brief         Dummy function, it returns just the passed node path.
*  \author        Sascha Kaden
*  \param[in,out] list of path nodes
*  \date          2017-05-23
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> DummyPathModifier<dim>::smoothPath(
    const std::vector<std::shared_ptr<Node<dim>>> &nodes) const {
    return nodes;
}

} /* namespace ippp */

#endif /* DUMMYPATHMODIFIER_HPP */
