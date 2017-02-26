//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef NORMALRRTPLANNER_HPP
#define NORMALRRTPLANNER_HPP

#include <mutex>

#include "RRTPlanner.hpp"

namespace rmpl {

/*!
* \brief   Class of the NormalRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class NormalRRTPlanner : public RRTPlanner<dim> {
  public:
    NormalRRTPlanner(const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions<dim> &options)
        : RRTPlanner<dim>("Normal RRT Planner", robot, options) {
    }

    bool connectGoalNode(Vector<dim> goal);

  protected:
    void computeRRTNode(const Vector<dim> &randVec, std::shared_ptr<Node<dim>> &newNode);

    std::mutex m_mutex;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_planner;
    using Planner<dim>::m_robot;
    using Planner<dim>::m_sampler;
    using RRTPlanner<dim>::m_initNode;
    using RRTPlanner<dim>::m_goalNode;
    using RRTPlanner<dim>::m_stepSize;
};

/*!
*  \brief         Computation of the new Node<dim> by the normal RRT algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
template <unsigned int dim>
void NormalRRTPlanner<dim>::computeRRTNode(const Vector<dim> &randVec, std::shared_ptr<Node<dim>> &newNode) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randVec);

    // compute Node<dim> new with fixed step size
    Vector<dim> newVec = this->computeNodeNew(randVec, nearestNode->getValues());
    newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));

    if (m_collision->controlVec(newNode->getValues())) {
        newNode = nullptr;
        return;
    } else if (!m_planner->controlTrajectory(newNode->getValues(), nearestNode->getValues())) {
        newNode = nullptr;
        return;
    }

    float edgeCost = this->m_heuristic->calcEdgeCost(nearestNode, newNode);
    newNode->setParent(nearestNode, edgeCost);

    m_mutex.lock();
    nearestNode->addChild(newNode, edgeCost);
    m_mutex.unlock();
}

/*!
*  \brief      Connects goal Node<dim> to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool NormalRRTPlanner<dim>::connectGoalNode(Vector<dim> goal) {
    if (m_collision->controlVec(goal)) {
        Logging::warning("Goal Node in collision", this);
        return false;
    }

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (auto node : nearNodes) {
        if (m_planner->controlTrajectory(goal, node->getValues())) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode, this->m_heuristic->calcEdgeCost(nearestNode, goalNode));
        m_goalNode = goalNode;
        m_graph->addNode(goalNode);
        // Logging::info("Goal Node<dim> is connected", this);
        m_pathPlanned = true;
        Logging::info("Goal could connected", this);
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}

} /* namespace rmpl */

#endif /* NORMALRRTPLANNER_HPP */
