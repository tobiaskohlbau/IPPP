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

#ifndef TREEPOSEEVALUATOR_HPP
#define TREEPOSEEVALUATOR_HPP

#include <string>
#include <vector>

#include <ippp/dataObj/Graph.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/modules/evaluator/Evaluator.hpp>
#include <ippp/util/Logging.h>
#include <ippp/util/UtilGeo.hpp>
#include <ippp/util/UtilPlanner.hpp>

namespace ippp {

/*!
* \brief   TreePoseEvaluator transforms the node configurations from the graph to the pose of the robot and checks there distance
* to the passed target poses. Return true at the evaluation step, if all targets are inside the distance of C.
* \author  Sascha Kaden
* \date    2017-09-30
*/
template <unsigned int dim>
class TreePoseEvaluator : public Evaluator<dim> {
  public:
    TreePoseEvaluator(const std::shared_ptr<Graph<dim>> &graph, const std::shared_ptr<Environment> &env,
                      const std::pair<Vector6, Vector6> &C, const std::string &name = "TreePoseEvaluator");

    bool evaluate();
    void initialize() override;
    void setPoses(const std::vector<Vector6> &targets) override;

  protected:
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<RobotBase> m_robot = nullptr;

    size_t m_lastNodeIndex = 0;
    std::vector<bool> m_validTargets;
    std::pair<Vector6, Vector6> m_C;

    using Evaluator<dim>::m_targetPoses;
};

/*!
*  \brief      Constructor of the class TreePoseEvaluator
*  \author     Sascha Kaden
*  \param[in]  DistanceMetric
*  \param[in]  Graph
*  \param[in]  Environment
*  \param[in]  min max distance of the pose
*  \date       2017-10-16
*/
template <unsigned int dim>
TreePoseEvaluator<dim>::TreePoseEvaluator(const std::shared_ptr<Graph<dim>> &graph, const std::shared_ptr<Environment> &env,
                                          const std::pair<Vector6, Vector6> &C, const std::string &name)
    : Evaluator<dim>(name), m_C(C), m_environment(env), m_graph(graph), m_robot(env->getRobot()) {
}

/*!
*  \brief      Evaluation of the new nodes inside of the graph by checking of the distance of the configruration to the target
* poses.
*  \author     Sascha Kaden
*  \param[out] Evaluation result, true if all targets are close to graph nodes
*  \date       2017-09-30
*/
template <unsigned int dim>
bool TreePoseEvaluator<dim>::evaluate() {
    for (size_t targetIndex = 0; targetIndex < m_targetPoses.size(); ++targetIndex) {
        if (m_validTargets[targetIndex])
            continue;

        const Vector6 &target = m_targetPoses[targetIndex];
        for (auto &node : m_graph->getNodes(m_lastNodeIndex)) {
            if (util::checkConfigToPose<dim>(node->getValues(), target, *m_robot, m_C)) {
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
void TreePoseEvaluator<dim>::initialize() {
    Logging::debug("Initialize", this);

    m_validTargets = std::vector<bool>(m_targetPoses.size(), false);
    m_lastNodeIndex = 0;
}

/*!
*  \brief      Set target poses for evaluation.
*  \author     Sascha Kaden
*  \param[in]  target poses
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

    m_targetPoses = targets;
    initialize();
}

} /* namespace ippp */

#endif /* TREEPOSEEVALUATOR_HPP */
