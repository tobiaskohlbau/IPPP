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

#ifndef TREECONNECTEVALUATOR_HPP
#define TREECONNECTEVALUATOR_HPP

#include <ippp/dataObj/Graph.hpp>
#include <ippp/modules/evaluator/Evaluator.hpp>
#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>
#include <ippp/modules/validityChecker/ValidityChecker.hpp>
#include <ippp/util/Logging.h>
#include <ippp/util/UtilPlanner.hpp>

namespace ippp {

/*!
* \brief   Evaluator interface.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class TreeConnectEvaluator : public Evaluator<dim> {
  public:
    TreeConnectEvaluator(const std::shared_ptr<Graph<dim>> &graphA, const std::shared_ptr<Graph<dim>> &graphB,
                         const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                         const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double range = 1);

    bool evaluate();

  protected:
    std::shared_ptr<Graph<dim>> m_graphA = nullptr;
    std::shared_ptr<Graph<dim>> m_graphB = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;

    double m_range;
    size_t m_lastIndexA;
    size_t m_lastIndexB;
    bool m_useA;
};

/*!
*  \brief      Constructor of the class TreeConfigEvaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  DistanceMetric
*  \param[in]  Graph
*  \param[in]  maximum distance
*  \date       2017-09-30
*/
template <unsigned int dim>
TreeConnectEvaluator<dim>::TreeConnectEvaluator(const std::shared_ptr<Graph<dim>> &graphA,
                                                const std::shared_ptr<Graph<dim>> &graphB,
                                                const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                                const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double range)
    : Evaluator<dim>("TreeConfigEvaluator"),
      m_graphA(graphA),
      m_graphB(graphB),
      m_trajectory(trajectory),
      m_validityChecker(validityChecker),
      m_range(range),
      m_lastIndexA(0),
      m_lastIndexB(0),
      m_useA(true) {
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph and checking of the distance to target node.
*  \author     Sascha Kaden
*  \param[out] Evaluation result
*  \date       2017-09-30
*/
template <unsigned int dim>
bool TreeConnectEvaluator<dim>::evaluate() {
    std::pair<std::shared_ptr<Node<dim>>, std::shared_ptr<Node<dim>>> nodes;
    if (m_useA) {
        nodes = util::findGraphConnection(*m_graphA, *m_graphB, *m_trajectory, *m_validityChecker, m_range, m_lastIndexA);
        m_lastIndexA = m_graphA->numNodes() - 1;
    } else {
        nodes = util::findGraphConnection(*m_graphB, *m_graphA, *m_trajectory, *m_validityChecker, m_range, m_lastIndexB);
        m_lastIndexB = m_graphB->numNodes() - 1;
    }

    if (nodes.first == nullptr)
        return false;

    Logging::info("Graphs are connectable.", this);
    return true;
}

} /* namespace ippp */

#endif /* TREECONNECTEVALUATOR_HPP */
