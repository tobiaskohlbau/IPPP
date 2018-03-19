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

#ifndef PRMCONFIGEVALUATOR_HPP
#define PRMCONFIGEVALUATOR_HPP

#include <ippp/dataObj/Graph.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/modules/distanceMetrics/L2Metric.hpp>
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
class PRMConfigEvaluator : public Evaluator<dim> {
  public:
    PRMConfigEvaluator(const std::shared_ptr<Graph<dim>> &graph, const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                       const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double range = 10);

    bool evaluate();
    void setConfigs(const std::vector<Vector<dim>> &targets) override;

  protected:
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;
    L2Metric<dim> m_metric;

    double m_range = 1;
    std::vector<bool> m_validTargets;

    using Evaluator<dim>::m_targetConfigs;
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
PRMConfigEvaluator<dim>::PRMConfigEvaluator(const std::shared_ptr<Graph<dim>> &graph,
                                            const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                            const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double range)
    : Evaluator<dim>("PRMConfigEvaluator"),
      m_graph(graph),
      m_trajectory(trajectory),
      m_validityChecker(validityChecker),
      m_range(range) {
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph and checking of the distance to target node.
*  \author     Sascha Kaden
*  \param[out] Evaluation result
*  \date       2017-09-30
*/
template <unsigned int dim>
bool PRMConfigEvaluator<dim>::evaluate() {
    for (size_t targetIndex = 0; targetIndex < m_targetConfigs.size(); ++targetIndex) {
        if (m_validTargets[targetIndex])
            continue;

        const Vector<dim> &target = m_targetConfigs[targetIndex];
        if (util::findNearValidNode<dim>(target, *m_graph, *m_trajectory, *m_validityChecker, m_range)) {
            m_validTargets[targetIndex] = true;
            Logging::debug("Target: " + std::to_string(targetIndex) + " is solved.", this);
        }
    }

    // check that all configuration are connectable to a node
    for (auto &validTarget : m_validTargets)
        if (!validTarget)
            return false;

    // check that path inside the graph is possible
    for (auto config = m_targetConfigs.begin(); config < m_targetConfigs.end() - 1; ++config) {
        auto start = util::findNearValidNode<dim>(*config, *m_graph, *m_trajectory, *m_validityChecker, m_range);
        auto goal = util::findNearValidNode<dim>(*(config + 1), *m_graph, *m_trajectory, *m_validityChecker, m_range);
        if (!util::aStar(start, goal, m_metric))
            return false;
    }

    Logging::info("Queries solved.", this);
    return true;
}

/*!
*  \brief      Set target nodes for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void PRMConfigEvaluator<dim>::setConfigs(const std::vector<Vector<dim>> &targets) {
    if (targets.empty()) {
        Logging::error("Empty target config list", this);
        return;
    }

    for (auto &target : targets) {
        if (util::empty<dim>(target)) {
            Logging::error("Empty target config", this);
            return;
        }
    }

    m_validTargets = std::vector<bool>(targets.size(), false);
    m_targetConfigs = targets;
}

} /* namespace ippp */

#endif /* PRMCONFIGEVALUATOR_HPP */
