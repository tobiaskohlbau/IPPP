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

#ifndef UTILPLANNER_HPP
#define UTILPLANNER_HPP

#include <ippp/core/dataObj/Graph.hpp>
#include <ippp/core/distanceMetrics/DistanceMetric.hpp>
#include <ippp/core/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {
namespace util {

/*!
*  \brief      Try to find nearest Node of the graph with a valid connection to the passed config
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[in]  nearest Node
*  \date       2016-08-09
*/
template <unsigned int dim>
static std::shared_ptr<Node<dim>> getNearestValidNode(const Vector<dim> &config, const std::shared_ptr<Graph<dim>> &graph,
                                               const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                               const std::shared_ptr<DistanceMetric<dim>> &metric, const double range) {
    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = graph->getNearNodes(config, range);
    double dist = std::numeric_limits<double>::max();
    for (auto &nearNode : nearNodes) {
        if (trajectory->checkTrajectory(config, nearNode->getValues()) &&
            metric->calcDist(config, nearNode->getValues()) < dist) {
            dist = metric->calcDist(config, nearNode->getValues());
            nearestNode = nearNode;
        }
    }
    return nearestNode;
}

/*!
*  \brief         Expands the openList of the A* algorithm from the childes of the passed Node
*  \author        Sascha Kaden
*  \param[in]     current node
*  \param[in,out] open list
*  \param[in,out] closed list
*  \param[in]     DistanceMetric
*  \date          2016-08-09
*/
template <unsigned int dim>
static void expandNode(const std::shared_ptr<Node<dim>> currentNode, std::vector<std::shared_ptr<Node<dim>>> &openList,
                std::vector<std::shared_ptr<Node<dim>>> &closedList, const std::shared_ptr<DistanceMetric<dim>> &metric) {
    double dist, edgeCost;
    for (auto successor : currentNode->getChildNodes()) {
        if (util::contains(closedList, successor))
            continue;

        edgeCost = metric->calcDist(currentNode, successor);
        dist = currentNode->getCost() + edgeCost;

        if (util::contains(openList, successor) && dist >= successor->getCost())
            continue;

        successor->setQueryParent(currentNode, edgeCost);
        successor->setCost(dist);
        if (!util::contains(openList, successor))
            openList.push_back(successor);
    }
}

/*!
*  \brief      A* algorithm to find best path
*  \details    Query parents of all nodes has to be cleared at the Graph.
*  \author     Sascha Kaden
*  \param[in]  source Node (start)
*  \param[in]  goal Node (goal)
*  \param[in]  DistanceMetric
*  \param[out] result of algorithm
*  \date       2016-08-09
*/
template <unsigned int dim>
static bool aStar(const std::shared_ptr<Node<dim>> sourceNode, const std::shared_ptr<Node<dim>> targetNode,
           const std::shared_ptr<DistanceMetric<dim>> &metric) {
    std::vector<std::shared_ptr<Node<dim>>> openList, closedList;

    std::vector<std::shared_ptr<Edge<dim>>> edges = sourceNode->getChildEdges();
    for (int i = 0; i < edges.size(); ++i) {
        edges[i]->getTarget()->setCost(edges[i]->getCost());
        edges[i]->getTarget()->setQueryParent(sourceNode, metric->calcDist(edges[i]->getTarget(), sourceNode));
        openList.push_back(edges[i]->getTarget());
    }
    closedList.push_back(sourceNode);

    int count = 0;
    std::shared_ptr<Node<dim>> currentNode;
    while (!openList.empty()) {
        currentNode = util::removeMinFromList(openList);

        if (currentNode == targetNode) {
            return true;
        }
        closedList.push_back(currentNode);
        ++count;

        expandNode<dim>(currentNode, openList, closedList, metric);
    }
    return false;
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILPLANNER_HPP
