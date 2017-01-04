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

#ifndef STARRRTPLANNER_H_
#define STARRRTPLANNER_H_

#include <mutex>

#include "RRTPlanner.hpp"

namespace rmpl {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class StarRRTPlanner : public RRTPlanner<dim> {
  public:
    StarRRTPlanner(const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions &options)
        : RRTPlanner<dim>("RRT* Planner", robot, options) {
    }

    bool connectGoalNode(Vector<dim> goal);

  protected:
    void computeRRTNode(const Vector<dim> &randVec, std::shared_ptr<Node<dim>> &newNode);
    void chooseParent(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode,
                      std::vector<std::shared_ptr<Node<dim>>> &nearNodes);
    void reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode,
                std::vector<std::shared_ptr<Node<dim>>> &nearNodes);

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
*  \brief         Computation of the new Node<dim> by the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
template <unsigned int dim>
void StarRRTPlanner<dim>::computeRRTNode(const Vector<dim> &randVec, std::shared_ptr<Node<dim>> &newNode) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randVec);
    // set Node<dim> new fix fixed step size of 10
    Vector<dim> newVec = this->computeNodeNew(randVec, nearestNode->getValues());
    newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));

    std::vector<std::shared_ptr<Node<dim>>> nearNodes;
    chooseParent(newNode, nearestNode, nearNodes);

    if (m_collision->controlVec(newNode->getValues())) {
        newNode = nullptr;
        return;
    } else if (!m_planner->controlTrajectory(newNode, nearestNode)) {
        newNode = nullptr;
        return;
    }

    newNode->setCost(Heuristic<dim>::calcEdgeCost(newNode, nearestNode) + nearestNode->getCost());
    newNode->setParent(nearestNode);
    m_mutex.lock();
    nearestNode->addChild(newNode);
    m_mutex.unlock();

    reWire(newNode, nearestNode, nearNodes);
}

/*!
*  \brief         Choose parent algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] nearest Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
template <unsigned int dim>
void StarRRTPlanner<dim>::chooseParent(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode,
                                       std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = m_graph->getNearNodes(newNode, m_stepSize);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost) {
            if (m_planner->controlTrajectory(newNode, nearNodes[i])) {
                nearestNodeCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
        }
    }
}

/*!
*  \brief         Rewire algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] parent Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
template <unsigned int dim>
void StarRRTPlanner<dim>::reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &parentNode,
                                 std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    float oldDist, newDist;
    for (auto nearNode : nearNodes) {
        if (nearNode != parentNode) {
            oldDist = nearNode->getCost();
            newDist = Heuristic<dim>::calcEdgeCost(nearNode, newNode) + newNode->getCost();
            if (newDist < oldDist && m_planner->controlTrajectory(nearNode, newNode)) {
                m_mutex.lock();
                nearNode->setCost(newDist);
                nearNode->setParent(newNode);
                m_mutex.unlock();
            }
        }
    }
}

/*!
*  \brief      Connects goal Node<dim> to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool StarRRTPlanner<dim>::connectGoalNode(Vector<dim> goal) {
    if (m_collision->controlVec(goal))
        return false;

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost && m_planner->controlTrajectory(goalNode, nearNodes[i])) {
            minCost = nearNodes[i]->getCost();
            nearestNode = nearNodes[i];
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode);
        goalNode->setCost(goalNode->getParentEdge()->getCost() + nearestNode->getCost());
        m_goalNode = goalNode;
        m_pathPlanned = true;
        return true;
    }

    Logging::info("Goal could NOT connected", this);
    return false;
}

} /* namespace rmpl */

#endif /* STARRRTPLANNER_H_ */
