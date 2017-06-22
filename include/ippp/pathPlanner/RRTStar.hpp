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

#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include <mutex>

#include <ippp/pathPlanner/RRT.hpp>

namespace ippp {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class RRTStar : public RRT<dim> {
  public:
    RRTStar(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
            const std::shared_ptr<Graph<dim>> &graph);

    bool connectGoalNode(const Vector<dim> goal);

  protected:
    std::shared_ptr<Node<dim>> computeRRTNode(const Vector<dim> &randVec);
    virtual void chooseParent(const Vector<dim> &newVec, std::shared_ptr<Node<dim>> &nearestNode,
                              std::vector<std::shared_ptr<Node<dim>>> &nearNodes);
    void reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode,
                std::vector<std::shared_ptr<Node<dim>>> &nearNodes);

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_sampling;
    using RRT<dim>::m_initNode;
    using RRT<dim>::m_goalNode;
    using RRT<dim>::m_stepSize;
    using RRT<dim>::m_mutex;
};

/*!
*  \brief      Standard constructor of the class StarRRTPlanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  options
*  \date       2017-02-19
*/
template <unsigned int dim>
RRTStar<dim>::RRTStar(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
                      const std::shared_ptr<Graph<dim>> &graph)
    : RRT<dim>(environment, options, graph, "RRT*") {
}

/*!
*  \brief      Computation of the new Node by the RRT* algorithm
*  \author     Sascha Kaden
*  \param[in]  random Vec
*  \param[in]  new Node
*  \date       2017-04-16
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRTStar<dim>::computeRRTNode(const Vector<dim> &randConfig) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randConfig);
    // set Node<dim> new fix fixed step size of 10
    Vector<dim> newConfig = this->computeNodeNew(randConfig, nearestNode->getValues());
    if (m_collision->checkConfig(newConfig)) {
        return nullptr;
    }
    std::vector<std::shared_ptr<Node<dim>>> nearNodes;
    chooseParent(randConfig, nearestNode, nearNodes);

    if (!m_trajectory->checkTrajectory(newConfig, nearestNode->getValues())) {
        return nullptr;
    }

    std::shared_ptr<Node<dim>> newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newConfig));
    double edgeCost = this->m_metric->calcDist(newNode, nearestNode);
    newNode->setCost(edgeCost + nearestNode->getCost());
    newNode->setParent(nearestNode, edgeCost);
    m_mutex.lock();
    nearestNode->addChild(newNode, edgeCost);
    m_mutex.unlock();

    reWire(newNode, nearestNode, nearNodes);
    return newNode;
}

/*!
*  \brief         Choose parent algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in]     new Node
*  \param[in,out] nearest Node
*  \param[out]    vector of nearest nodes
*  \date          2017-04-16
*/
template <unsigned int dim>
void RRTStar<dim>::chooseParent(const Vector<dim> &newConfig, std::shared_ptr<Node<dim>> &nearestNode,
                                std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = m_graph->getNearNodes(newConfig, m_stepSize);

    double nearestNodeCost = nearestNode->getCost();
    for (auto nearNode : nearNodes) {
        if (nearNode->getCost() < nearestNodeCost) {
            if (m_trajectory->checkTrajectory(newConfig, nearNode->getValues())) {
                nearestNodeCost = nearNode->getCost();
                nearestNode = nearNode;
            }
        }
    }
}

/*!
*  \brief         Rewire algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] parent Node
*  \param[in,out] vector of nearest nodes
*  \date          2016-06-02
*/
template <unsigned int dim>
void RRTStar<dim>::reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &parentNode,
                          std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    double oldDist, newDist, edgeCost;
    for (auto nearNode : nearNodes) {
        if (nearNode != parentNode) {
            edgeCost = this->m_metric->calcDist(nearNode, newNode);
            oldDist = nearNode->getCost();
            newDist = edgeCost + newNode->getCost();
            if (newDist < oldDist && m_trajectory->checkTrajectory(nearNode, newNode)) {
                m_mutex.lock();
                nearNode->setCost(newDist);
                nearNode->setParent(newNode, edgeCost);
                m_mutex.unlock();
            }
        }
    }
}

/*!
*  \brief      Connects goal Node to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRTStar<dim>::connectGoalNode(Vector<dim> goal) {
    if (m_collision->checkConfig(goal)) {
        Logging::warning("Goal Node in collision", this);
        return false;
    }

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    double minCost = std::numeric_limits<double>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost && m_trajectory->checkTrajectory(goalNode, nearNodes[i])) {
            minCost = nearNodes[i]->getCost();
            nearestNode = nearNodes[i];
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode, this->m_metric->calcDist(goalNode, nearestNode));
        goalNode->setCost(goalNode->getParentEdge()->getCost() + nearestNode->getCost());
        m_goalNode = goalNode;
        m_pathPlanned = true;
        Logging::info("Goal could connected", this);
        return true;
    }

    Logging::warning("Goal could NOT connected", this);
    return false;
}

} /* namespace ippp */

#endif /* RRTSTAR_HPP */
