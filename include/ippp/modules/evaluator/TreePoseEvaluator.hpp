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

#ifndef TREEPOSEEVALUATOR_HPP
#define TREEPOSEEVALUATOR_HPP

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
class TreePoseEvaluator : public Evaluator<dim> {
  public:
    TreePoseEvaluator(const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Graph<dim>> &graph,
                      const std::shared_ptr<Environment> &env, double dist = 10);

    bool evaluate();
    void setPoses(const std::vector<Vector6> &targets) override;

  protected:
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;

    double m_dist = 1;
    double m_simplifiedDist;
    size_t m_lastNodeIndex = 0;
    std::vector<bool> m_validTargets;

    using Evaluator<dim>::m_targetPoses;
};

/*!
*  \brief      Constructor of the class TreeQueryEvaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  DistanceMetric
*  \param[in]  Graph
*  \param[in]  maximum distance
*  \date       2017-09-30
*/
template <unsigned int dim>
TreePoseEvaluator<dim>::TreePoseEvaluator(const std::shared_ptr<DistanceMetric<dim>> &metric,
                                          const std::shared_ptr<Graph<dim>> &graph, const std::shared_ptr<Environment> &env,
                                          double dist)
    : Evaluator<dim>("TreePoseEvaluator"),
      m_graph(graph),
      m_metric(metric),
      m_environment(env),
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
bool TreePoseEvaluator<dim>::evaluate() {
    for (size_t targetIndex = 0; targetIndex < m_targetConfigs.size(); ++targetIndex) {
        if (m_validTargets[targetIndex])
            continue;

        const Vector<dim> &target = m_targetConfigs[targetIndex];
        bool found = false;
        for (size_t index = m_lastNodeIndex; index < m_graph->numNodes(); ++index) {
            const Vector<dim> &nodeConfig = m_graph->getNode(index)->getValues();
            if (m_metric->calcSimpleDist(target, nodeConfig) < m_simplifiedDist &&
                m_validityChecker->check(m_trajectory->calcTrajBin(target, nodeConfig))) {
                found = true;
                m_validTargets[targetIndex] = true;
                Logging::debug("Target: " + std::to_string(targetIndex) + " is solved.", this);
                break;
            }
        }
    }

    for (auto validTarget : m_validTargets)
        if (!validTarget)
            return false;

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
void TreePoseEvaluator<dim>::setPoses(const std::vector<Vector6> &targets) {
    if (targets.empty()) {
        Logging::error("Empty target pose list", this);
        return;
    }

    for (auto &target : targets) {
        if (util::empty<6>(target)) {
            Logging::error("Empty target pose", this);
            return;
        }
    }

    m_validTargets = std::vector<bool>(targets.size(), false);
    m_targetPoses = targets;
}

} /* namespace ippp */

#endif /* TREEPOSEEVALUATOR_HPP */
