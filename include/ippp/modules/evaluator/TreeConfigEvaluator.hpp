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

#ifndef TREECONFIGEVALUATOR_HPP
#define TREECONFIGEVALUATOR_HPP

#include <ippp/dataObj/Graph.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/modules/distanceMetrics/DistanceMetric.hpp>
#include <ippp/modules/evaluator/Evaluator.hpp>
#include <ippp/modules/trajectoryPlanner/TrajectoryPlanner.hpp>
#include <ippp/modules/validityChecker/ValidityChecker.hpp>
#include <ippp/util/Logging.h>

namespace ippp {

/*!
* \brief   Evaluator interface.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class TreeConfigEvaluator : public Evaluator<dim> {
  public:
    TreeConfigEvaluator(const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Graph<dim>> &graph,
                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                        const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double dist = 10);

    bool evaluate();
    void initialize() override;
    void setConfigs(const std::vector<Vector<dim>> &targets) override;

  protected:
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;

    double m_dist = 1;
    double m_simplifiedDist;
    size_t m_lastNodeIndex = 0;
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
TreeConfigEvaluator<dim>::TreeConfigEvaluator(const std::shared_ptr<DistanceMetric<dim>> &metric,
                                              const std::shared_ptr<Graph<dim>> &graph,
                                              const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                              const std::shared_ptr<ValidityChecker<dim>> &validityChecker, double dist)
    : Evaluator<dim>("TreeConfigEvaluator"),
      m_graph(graph),
      m_metric(metric),
      m_trajectory(trajectory),
      m_validityChecker(validityChecker),
      m_dist(dist),
      m_simplifiedDist(dist) {
    m_metric->simplifyDist(m_simplifiedDist);
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph and checking of the distance to target node.
*  \author     Sascha Kaden
*  \param[out] Evaluation result
*  \date       2017-09-30
*/
template <unsigned int dim>
bool TreeConfigEvaluator<dim>::evaluate() {
    for (size_t targetIndex = 0; targetIndex < m_targetConfigs.size(); ++targetIndex) {
        if (m_validTargets[targetIndex])
            continue;

        const Vector<dim> &target = m_targetConfigs[targetIndex];
        for (auto &node : m_graph->getNodes(m_lastNodeIndex)) {
            const Vector<dim> &nodeConfig = node->getValues();
            if (m_metric->calcSimpleDist(target, nodeConfig) < m_simplifiedDist &&
                m_validityChecker->check(m_trajectory->calcTrajBin(target, nodeConfig))) {
                m_validTargets[targetIndex] = true;
                Logging::debug("Target: " + std::to_string(targetIndex) + " is solved.", this);
                break;
            }
        }
    }
    m_lastNodeIndex = m_graph->numNodes() - 1;

    for (auto validTarget : m_validTargets)
        if (!validTarget)
            return false;

    Logging::info("Queries solved.", this);
    return true;
}

template <unsigned int dim>
void TreeConfigEvaluator<dim>::initialize() {
    m_lastNodeIndex = 0;
    m_validTargets = std::vector<bool>(m_targetConfigs.size(), false);
}

/*!
*  \brief      Set target nodes for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void TreeConfigEvaluator<dim>::setConfigs(const std::vector<Vector<dim>> &targets) {
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

#endif /* TREECONFIGEVALUATOR_HPP */
