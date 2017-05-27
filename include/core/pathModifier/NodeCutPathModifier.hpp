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

#ifndef NODECUTPATHMODIFIER_HPP
#define NODECUTPATHMODIFIER_HPP

#include <core/pathModifier/PathModifier.hpp>

namespace ippp {

/*!
* \brief   Base class of all path modifier or path smoother.
* \author  Sascha Kaden
* \date    2017-05-23
*/
template <unsigned int dim>
class NodeCutPathModifier : public PathModifier<dim> {
  public:
    NodeCutPathModifier(const std::shared_ptr<Environment> &environment, std::shared_ptr<CollisionDetection<dim>> &collision,
                        std::shared_ptr<TrajectoryPlanner<dim>> &trajectory);

    std::vector<std::shared_ptr<Node<dim>>> smoothPath(const std::vector<std::shared_ptr<Node<dim>>> &nodes) const;

  protected:
    std::shared_ptr<CollisionDetection<dim>> m_collision = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
};

/*!
*  \brief      Constructor of the base Sampler class
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionDetection
*  \param[in]  TrajectoryPlanner
*  \date       2017-05-23
*/
template <unsigned int dim>
NodeCutPathModifier<dim>::NodeCutPathModifier(const std::shared_ptr<Environment> &environment,
                                              std::shared_ptr<CollisionDetection<dim>> &collision,
                                              std::shared_ptr<TrajectoryPlanner<dim>> &trajectory)
    : PathModifier<dim>(environment, collision, trajectory, "Node cutter") {
}

/*!
*  \brief      Try to cut nodes from the path, if a free path to the following node valid.
*  \author     Sascha Kaden
*  \param[in]  list of path nodes
*  \param[out] shorted node path
*  \date       2017-05-23
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> NodeCutPathModifier<dim>::smoothPath(
    const std::vector<std::shared_ptr<Node<dim>>> &nodes) const {
    std::vector<std::shared_ptr<Node<dim>>> smoothedNodes = nodes;
    unsigned int i = 0;
    auto countNodes = smoothedNodes.size() - 2;
    while (i < countNodes) {
        while (i < countNodes &&
               m_trajectory->controlTrajectory(smoothedNodes[i]->getValues(), smoothedNodes[i + 2]->getValues())) {
            smoothedNodes.erase(smoothedNodes.begin() + i + 1);
            --countNodes;
        }
        ++i;
    }
    return smoothedNodes;
}

} /* namespace ippp */

#endif /* NODECUTPATHMODIFIER_HPP */
