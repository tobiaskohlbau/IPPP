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

#ifndef RRTSTARCONNECT_HPP
#define RRTSTARCONNECT_HPP

#include <ippp/planner/RRTStar.hpp>

namespace ippp {

/*!
* \brief   Class of the StarRRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class RRTStarConnect : public RRTStar<dim> {
  public:
    RRTStarConnect(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
                   const std::shared_ptr<Graph<dim>> &graphA, const std::shared_ptr<Graph<dim>> &graphB,
                   const std::string &name = "RRTStarConnect");

    virtual bool computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads = 1) override;

  protected:
    bool initNodes(const Vector<dim> &start, const Vector<dim> &goal);

    std::shared_ptr<Graph<dim>> m_graphA = nullptr;
    std::shared_ptr<Graph<dim>> m_graphB = nullptr;

    using Planner<dim>::m_validityChecker;
    using Planner<dim>::m_evaluator;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_metric;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_plannerCollector;
    using Planner<dim>::m_sampling;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::updateStats;
    using Planner<dim>::initParams;
    using TreePlanner<dim>::setInitNode;
    using TreePlanner<dim>::connectGoalNode;
    using RRT<dim>::m_initNode;
    using RRT<dim>::m_goalNode;
    using RRT<dim>::m_stepSize;
};

/*!
*  \brief      Standard constructor of the class StarRRTPlanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  options
*  \date       2017-02-19
*/
template <unsigned int dim>
RRTStarConnect<dim>::RRTStarConnect(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
                                    const std::shared_ptr<Graph<dim>> &graphA, const std::shared_ptr<Graph<dim>> &graphB,
                                    const std::string &name)
    : RRTStar<dim>(environment, options, graphA, name), m_graphA(graphA), m_graphB(graphB) {
}

template <unsigned int dim>
bool RRTStarConnect<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads) {
    m_plannerCollector->startPlannerTimer();
    if (!initNodes(start, goal))
        return false;

    // tmp start and goal config for sampler parameter
    auto tmpStart = start;
    auto tmpGoal = goal;
    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        initParams(tmpStart, tmpGoal);
        m_sampling->setGraph(m_graph);
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        this->expand(numNodes, numThreads);
        std::swap(m_graph, m_graphB);
        std::swap(tmpStart, tmpGoal);
    }

    // swap graphs to original state und search for the connection
    if (m_graph != m_graphA) {
        m_graphB = m_graph;
        m_graph = m_graphA;
        m_sampling->setGraph(m_graph);
    }
    std::pair<std::shared_ptr<Node<dim>>, std::shared_ptr<Node<dim>>> nodes =
        util::findGraphConnection(*m_graph, *m_graphB, *m_trajectory, *m_validityChecker, m_stepSize);
    if (nodes.first == nullptr) {
        Logging::warning("Found no connection between trees!", this);
        return false;
    }

    auto treeNode = nodes.first;
    auto pathNode = nodes.second;
    double dist;
    while (pathNode != nullptr) {
        dist = m_metric->calcDist(*treeNode, *pathNode);
        auto tmpNode = std::make_shared<Node<dim>>(pathNode->getValues());
        tmpNode->setParent(treeNode, dist);
        tmpNode->setCost(treeNode->getCost() + dist);
        m_graph->addNode(tmpNode);

        treeNode->addChild(tmpNode, dist);
        pathNode = pathNode->getParentNode();
        treeNode = tmpNode;
    }
    m_goalNode = treeNode;
    m_pathPlanned = true;

    // stats
    updateStats();
    m_graphA->updateStats();
    m_graphB->updateStats();
    m_plannerCollector->stopPlannerTimer();

    Logging::info("Found path", this);
    return m_pathPlanned;
}

/*!
*  \brief      Set init Node of the TreePlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if valid
*  \date       2017-06-20
*/
template <unsigned int dim>
bool RRTStarConnect<dim>::initNodes(const Vector<dim> &start, const Vector<dim> &goal) {
    m_evaluator->initialize();

    if (m_initNode && m_goalNode) {
        if (start == m_initNode->getValues() && goal == m_goalNode->getValues()) {
            Logging::debug("Equal start and goal configuration, tree will be expanded", this);
            return true;
        } else {
            Logging::info("Trees will be cleared", this);
            m_graph->clear();
            m_graphB->clear();
        }
    }

    if (!m_validityChecker->check(start)) {
        Logging::warning("Init configuration could not be connected", this);
        return false;
    }

    if (!m_validityChecker->check(goal)) {
        Logging::warning("Goal configuration could not be connected", this);
        return false;
    }

    m_initNode = std::make_shared<Node<dim>>(start);
    m_goalNode = std::make_shared<Node<dim>>(goal);
    m_graph->addNode(m_initNode);
    m_graphB->addNode(m_goalNode);
    return true;
}

} /* namespace ippp */

#endif /* RRTSTARCONNECT_HPP */
