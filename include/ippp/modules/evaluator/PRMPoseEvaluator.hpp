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

#ifndef PRMPOSEEVALUATOR_HPP
#define PRMPOSEEVALUATOR_HPP

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
class PRMPoseEvaluator : public Evaluator<dim> {
  public:
    PRMPoseEvaluator(const std::shared_ptr<Environment> &env, const std::shared_ptr<Graph<dim>> &graph,
                     const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                     const std::shared_ptr<ValidityChecker<dim>> &validityChecker, const std::pair<Vector6, Vector6> &C,
                     double range = 10);

    bool evaluate();
    void setConfigs(const std::vector<Vector<dim>> &targets) override;
    void setPoses(const std::vector<Vector6> &targets) override;

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;
    std::shared_ptr<RobotBase> m_robot = nullptr;
    L2Metric<dim> m_metric;

    double m_range = 1;
    size_t m_lastNodeIndex = 0;
    std::pair<Vector6, Vector6> m_C;
    std::vector<std::shared_ptr<Node<dim>>> m_targetConfigNodes;
    std::vector<std::shared_ptr<Node<dim>>> m_targetPoseNodes;

    using Evaluator<dim>::m_targetConfigs;
    using Evaluator<dim>::m_targetPoses;
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
PRMPoseEvaluator<dim>::PRMPoseEvaluator(const std::shared_ptr<Environment> &env, const std::shared_ptr<Graph<dim>> &graph,
                                        const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                        const std::shared_ptr<ValidityChecker<dim>> &validityChecker,
                                        const std::pair<Vector6, Vector6> &C, double range)
    : Evaluator<dim>("PRMPoseEvaluator"),
      m_C(C),
      m_environment(env),
      m_graph(graph),
      m_lastNodeIndex(0),
      m_trajectory(trajectory),
      m_validityChecker(validityChecker),
      m_robot(env->getRobot()),
      m_range(range) {
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph and checking of the distance to target node.
*  \author     Sascha Kaden
*  \param[out] Evaluation result
*  \date       2017-09-30
*/
template <unsigned int dim>
bool PRMPoseEvaluator<dim>::evaluate() {
    if (m_graph->empty())
        return false;
    // check configs
    for (size_t index = 0; index < m_targetConfigs.size(); ++index) {
        if (m_targetConfigNodes[index])
            continue;

        auto validNode =
            util::findNearValidNode<dim>(m_targetConfigs[index], *m_graph, *m_trajectory, *m_validityChecker, m_range);
        if (validNode) {
            m_targetConfigNodes[index] = validNode;
            Logging::debug("Config: " + std::to_string(index) + " is solved.", this);
        }
    }

    // check poses
    for (size_t index = 0; index < m_targetPoses.size(); ++index) {
        if (m_targetPoseNodes[index])
            continue;

        const Vector6 &target = m_targetPoses[index];
        for (auto &node : m_graph->getNodes(m_lastNodeIndex)) {
            const Vector<dim> &nodeConfig = node->getValues();
            if (util::checkConfigToPose<dim>(nodeConfig, target, *m_robot, m_C)) {
                m_targetPoseNodes[index] = node;
                Logging::debug("Pose: " + std::to_string(index) + " is solved.", this);
                break;
            }
        }
    }

    m_lastNodeIndex = m_graph->numNodes() - 1;

    // check that all configuration and poses are connected to a node
    for (auto &config : m_targetConfigNodes)
        if (!config)
            return false;
    for (auto &pose : m_targetPoseNodes)
        if (!pose)
            return false;

    // check that path inside the graph is possible
    auto nodes = m_targetConfigNodes;
    nodes.insert(nodes.begin(), m_targetPoseNodes.begin(), m_targetPoseNodes.end());
    for (auto node = nodes.begin(); node < nodes.end() - 1; ++node)
        if (!util::aStar<dim>(*node, *(node + 1), m_metric))
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
void PRMPoseEvaluator<dim>::setConfigs(const std::vector<Vector<dim>> &targets) {
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

    m_targetConfigNodes = std::vector<std::shared_ptr<Node<dim>>>(targets.size(), nullptr);
    m_targetConfigs = targets;
}

/*!
*  \brief      Set target nodes for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target Nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
void PRMPoseEvaluator<dim>::setPoses(const std::vector<Vector6> &targets) {
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

    m_targetPoseNodes = std::vector<std::shared_ptr<Node<dim>>>(targets.size(), nullptr);
    m_targetPoses = targets;
}

} /* namespace ippp */

#endif /* PRMCONFIGEVALUATOR_HPP */
