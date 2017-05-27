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

#ifndef RRTSTARCONTTRAJ_HPP
#define RRTSTARCONTTRAJ_HPP

#include <pathPlanner/RRTStar.hpp>

namespace ippp {

/*!
* \brief   Class of the RRTStarContTraj, adaption of the RRT* path planner.
* \details Choose parent with the smallest cost and test the trajectory continuous to the new Node. Thereby the last valid point
* will be added to the graph.
* \author  Sascha Kaden
* \date    2017-04-16
*/
template <unsigned int dim>
class RRTStarContTraj : public RRTStar<dim> {
  public:
    RRTStarContTraj(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options);

  protected:
    std::shared_ptr<Node<dim>> computeRRTNode(const Vector<dim> &randVec);
    void chooseParent(const Vector<dim> &newVec, std::shared_ptr<Node<dim>> &nearestNode,
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
*  \brief      Standard constructor of the class RRTStarContTraj
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  options
*  \date       2017-02-19
*/
template <unsigned int dim>
RRTStarContTraj<dim>::RRTStarContTraj(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options)
    : RRTStar<dim>(environment, options) {
}

/*!
*  \brief         Computation of the new Node by the RRT* algorithm with small differences, see constructor description.
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[out]    new Node
*  \date          2016-06-02
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRTStarContTraj<dim>::computeRRTNode(const Vector<dim> &randVec) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randVec);
    // set Node<dim> new fix fixed step size of 10
    Vector<dim> newVec = this->computeNodeNew(randVec, nearestNode->getValues());
    std::vector<std::shared_ptr<Node<dim>>> nearNodes;
    chooseParent(newVec, nearestNode, nearNodes);

    newVec = m_trajectory->controlTrajCont(nearestNode->getValues(), newVec);

    if (util::empty<dim>(newVec)) {
        return nullptr;
    }

    std::shared_ptr<Node<dim>> newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));
    double edgeCost = this->m_metric->calcEdgeCost(newNode, nearestNode);
    newNode->setCost(edgeCost + nearestNode->getCost());
    newNode->setParent(nearestNode, edgeCost);
    m_mutex.lock();
    nearestNode->addChild(newNode, edgeCost);
    m_mutex.unlock();

    // reWire(newNode, nearestNode, nearNodes);
    return newNode;
}

/*!
*  \brief         Choose parent with the lowest cost
*  \author        Sascha Kaden
*  \param[in]     new Node
*  \param[in,out] nearest Node
*  \param[out]    vector of nearest nodes
*  \date          2017-04-16
*/
template <unsigned int dim>
void RRTStarContTraj<dim>::chooseParent(const Vector<dim> &newVec, std::shared_ptr<Node<dim>> &nearestNode,
                                        std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = m_graph->getNearNodes(newVec, m_stepSize);

    double nearestNodeCost = nearestNode->getCost();
    for (auto nearNode : nearNodes) {
        if (nearNode->getCost() < nearestNodeCost) {
            nearestNodeCost = nearNode->getCost();
            nearestNode = nearNode;
        }
    }
}

} /* namespace ippp */

#endif /* RRTSTARCONTTRAJ_HPP */
