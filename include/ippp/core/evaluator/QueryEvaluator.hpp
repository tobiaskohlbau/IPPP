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

#ifndef QUERYEVALUATOR_HPP
#define QUERYEVALUATOR_HPP

#include <ippp/core/dataObj/Graph.hpp>
#include <ippp/core/distanceMetrics/DistanceMetric.hpp>
#include <ippp/core/evaluator/Evaluator.hpp>
#include <ippp/core/util/UtilVec.hpp>

namespace ippp {

/*!
* \brief   Evaluator interface.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class QueryEvaluator : public Evaluator<dim> {
  public:
    QueryEvaluator(const std::shared_ptr<Environment> &environment, const std::shared_ptr<DistanceMetric<dim>> &metric,
                   const std::shared_ptr<Graph<dim>> &graph, double dist = 1);

    bool evaluate();
    void setQuery(const std::vector<Vector<dim>> &targets) override;

  protected:
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;

    double m_dist = 1;
    size_t m_lastNodeIndex = 0;
    std::vector<bool> m_validTargets;

    using Evaluator<dim>::m_targets;
};

/*!
*  \brief      Constructor of the class Evaluator
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  DistanceMetric
*  \param[in]  Graph
*  \param[in]  maximum distance
*  \date       2017-09-30
*/
template <unsigned int dim>
QueryEvaluator<dim>::QueryEvaluator(const std::shared_ptr<Environment> &environment,
                                    const std::shared_ptr<DistanceMetric<dim>> &metric, const std::shared_ptr<Graph<dim>> &graph,
                                    double dist)
    : Evaluator<dim>("QueryEvaluator", environment), m_graph(graph), m_metric(metric), m_dist(dist) {
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph and checking of the distance to target node.
*  \author     Sascha Kaden
*  \param[out] Evaluation result
*  \date       2017-09-30
*/
template <unsigned int dim>
bool QueryEvaluator<dim>::evaluate() {
    bool allFound = true;
    for (size_t targetIndex = 0; targetIndex < m_targets.size(); ++targetIndex) {
        if (m_validTargets[targetIndex])
            continue;

        bool found = false;
        for (size_t index = m_lastNodeIndex; index < m_graph->size(); ++index) {
            if (m_metric->calcDist(m_targets[targetIndex], m_graph->getNode(index)->getValues()) < m_dist) {
                found = true;
                m_validTargets[targetIndex] = true;
                break;
            }
        }
        if (!found)
            allFound = false;
    }
    if (!allFound) {
        m_lastNodeIndex = m_graph->size() - 1;
        return false;
    }

    return true;
}

/*!
*  \brief      Set target nodes for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void QueryEvaluator<dim>::setQuery(const std::vector<Vector<dim>> &targets) {
    if (targets.empty())
        return;

    for (auto &target : targets)
        if (util::empty<dim>(target))
            return;

    m_validTargets = std::vector<bool>(targets.size(), false);
    m_targets = targets;
}

} /* namespace ippp */

#endif /* QUERYEVALUATOR_HPP */