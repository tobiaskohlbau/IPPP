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

#ifndef NODECUTPATHMODIFIER_HPP
#define NODECUTPATHMODIFIER_HPP

#include <ippp/modules/pathModifier/PathModifier.hpp>

namespace ippp {

/*!
* \brief   NodeCutPathModifier smooths the path by cutting unneeded nodes from the passed path.
* \author  Sascha Kaden
* \date    2017-05-23
*/
template <unsigned int dim>
class NodeCutPathModifier : public PathModifier<dim> {
  public:
    NodeCutPathModifier(const std::shared_ptr<Environment> &environment,

                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                        const std::shared_ptr<ValidityChecker<dim>> &validityChecker);

    std::vector<std::shared_ptr<Node<dim>>> smoothPath(const std::vector<std::shared_ptr<Node<dim>>> &nodes) const;

  protected:
    using PathModifier<dim>::m_validityChecker;
    using PathModifier<dim>::m_environment;
    using PathModifier<dim>::m_trajectory;
};

template <unsigned int dim>
NodeCutPathModifier<dim>::NodeCutPathModifier(const std::shared_ptr<Environment> &environment,
                                              const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                              const std::shared_ptr<ValidityChecker<dim>> &validityChecker)
    : PathModifier<dim>("NodeCut", environment, trajectory, validityChecker) {
}

/*!
*  \brief      Tries to cut nodes from the path.
*  \details    Tries to find a path between Node A and C, if possible Node B will be cut. This will be tried for the complete
* path.
*  \author     Sascha Kaden
*  \param[in]  list of path nodes
*  \param[out] shorted node path
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> NodeCutPathModifier<dim>::smoothPath(
    const std::vector<std::shared_ptr<Node<dim>>> &nodes) const {
    std::vector<std::shared_ptr<Node<dim>>> smoothedNodes = nodes;

    auto i = std::begin(smoothedNodes);
    while (i != std::end(smoothedNodes) - 2) {
        auto j = i + 2;
        while (j != std::end(smoothedNodes) - 1) {
            if (m_validityChecker->check(m_trajectory->calcTrajBin(**i, **j))) {
                j = smoothedNodes.erase(j - 1);
                ++j;
            } else {
                break;
            }
        }
        ++i;
    }
    // try to short cut the foreleast node
    if (smoothedNodes.size() > 2 &&
        m_validityChecker->check(m_trajectory->calcTrajBin(smoothedNodes[smoothedNodes.size() - 3]->getValues(),
                                                           smoothedNodes[smoothedNodes.size() - 1]->getValues())))
        smoothedNodes.erase(std::end(smoothedNodes) - 2);

    return smoothedNodes;
}

} /* namespace ippp */

#endif /* NODECUTPATHMODIFIER_HPP */
