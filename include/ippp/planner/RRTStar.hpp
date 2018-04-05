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

#include <limits>

#include <ippp/planner/RRT.hpp>

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
            const std::shared_ptr<Graph<dim>> &graph, const std::string &name = "RRTStar");

    bool connectGoalNode(const Vector<dim> goal) override;

  protected:
    std::shared_ptr<Node<dim>> computeRRTNode(const Vector<dim> &randVec);
    virtual std::shared_ptr<Node<dim>> chooseParent(const Vector<dim> &newVec,
                                                    const std::vector<std::shared_ptr<Node<dim>>> &nearNodes);
    void reWire(std::shared_ptr<Node<dim>> &newNode, std::shared_ptr<Node<dim>> &nearestNode,
                const std::vector<std::shared_ptr<Node<dim>>> &nearNodes);

    using Planner<dim>::m_validityChecker;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_metric;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
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
                      const std::shared_ptr<Graph<dim>> &graph, const std::string &name)
    : RRT<dim>(environment, options, graph, name) {
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
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randConfig);

    Vector<dim> newConfig = this->computeNodeNew(randConfig, nearestNode->getValues());
    if (!m_validityChecker->check(newConfig))
        return nullptr;

    auto nearNodes = m_graph->getNearNodes(newConfig, m_stepSize);
    nearNodes.push_back(nearestNode);
    nearestNode = chooseParent(randConfig, nearNodes);

    if (!nearestNode)
        return nullptr;

    auto newNode = std::make_shared<Node<dim>>(newConfig);
    double edgeCost = m_metric->calcDist(*newNode, *nearestNode);
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
*  \param[in]     vector of nearest nodes
*  \date          2017-04-16
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRTStar<dim>::chooseParent(const Vector<dim> &newConfig,
                                                      const std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    double nearestNodeCost = std::numeric_limits<double>::max();
    for (auto &nearNode : nearNodes) {
        if (nearNode->getCost() < nearestNodeCost &&
            m_validityChecker->check(m_trajectory->calcTrajBin(newConfig, nearNode->getValues()))) {
            nearestNodeCost = nearNode->getCost();
            nearestNode = nearNode;
        }
    }
    return nearestNode;
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
                          const std::vector<std::shared_ptr<Node<dim>>> &nearNodes) {
    double oldDist, newDist, edgeCost;
    for (auto &nearNode : nearNodes) {
        if (nearNode != parentNode) {
            edgeCost = m_metric->calcDist(*nearNode, *newNode);
            oldDist = nearNode->getCost();
            newDist = edgeCost + newNode->getCost();
            if (newDist < oldDist && m_validityChecker->check(m_trajectory->calcTrajBin(*nearNode, *newNode))) {
                m_mutex.lock();
                nearNode->setCost(newDist);
                nearNode->setParent(newNode, edgeCost);
                m_mutex.unlock();
            }
        }
    }
}

/*!
*  \brief      Connects goal Node to tree, if connection is valid
*  \author     Sascha Kaden
*  \param[in]  goal configuration
*  \param[out] true, if the connection is valid
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRTStar<dim>::connectGoalNode(Vector<dim> goal) {
    if (!m_validityChecker->check(goal)) {
        Logging::warning("Goal Node in collision", this);
        return false;
    }

    auto nearestNode =
        util::getNearestValidNode<dim>(goal, *m_graph, *m_validityChecker, *m_trajectory, *m_metric, m_stepSize * 2);

    if (nearestNode) {
        std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
        goalNode->setParent(nearestNode, m_metric->calcDist(*goalNode, *nearestNode));
        goalNode->setCost(goalNode->getParentEdge().second + nearestNode->getCost());
        nearestNode->addChild(goalNode, m_metric->calcDist(*goalNode, *nearestNode));

        m_goalNode = goalNode;
        m_pathPlanned = true;
        Logging::info("Goal is connected", this);
        return true;
    } else {
        Logging::warning("Goal could NOT connected", this);
        return false;
    }
}

} /* namespace ippp */

#endif /* RRTSTAR_HPP */
