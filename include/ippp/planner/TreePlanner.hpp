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

#ifndef TREETPLANNER_HPP
#define TREETPLANNER_HPP

#include <mutex>

#include <ippp/planner/Planner.hpp>
#include <ippp/planner/options/RRTOptions.hpp>

namespace ippp {

/*!
* \brief   Super class of all tree path planners like RRT and EST.
* \author  Sascha Kaden
* \date    2017-06-20
*/
template <unsigned int dim>
class TreePlanner : public Planner<dim> {
  public:
    TreePlanner(const std::string &name, const std::shared_ptr<Environment> &environment, const PlannerOptions<dim> &options,
                const std::shared_ptr<Graph<dim>> &graph);

    virtual bool computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads);
    virtual bool computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, size_t numNodes, size_t numThreads);
    virtual bool expand(size_t numNodes, size_t numThreads);
    virtual bool setInitNode(const Vector<dim> start);

    virtual bool computeTree(size_t nbOfNodes, size_t nbOfThreads = 1) = 0;
    virtual bool connectGoalNode(const Vector<dim> goal) = 0;

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(double posRes = 1, double oriRes = 0.1);
    std::shared_ptr<Node<dim>> getInitNode() const;
    std::shared_ptr<Node<dim>> getGoalNode() const;

  protected:
    std::shared_ptr<Node<dim>> m_initNode = nullptr;
    std::shared_ptr<Node<dim>> m_goalNode = nullptr;

    using Planner<dim>::m_validityChecker;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_evaluator;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
};

/*!
*  \brief      Constructor of the class TreePlanner, base class of RRT and EST.
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  Environment
*  \param[in]  PlannerOptions
*  \param[in]  Graph
*  \date       2017-06-20
*/
template <unsigned int dim>
TreePlanner<dim>::TreePlanner(const std::string &name, const std::shared_ptr<Environment> &environment,
                              const PlannerOptions<dim> &options, const std::shared_ptr<Graph<dim>> &graph)
    : Planner<dim>(name, environment, options, graph) {
}

/*!
*  \brief      Compute path from start Node<dim> to goal Node<dim> with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2017-06-20
*/
template <unsigned int dim>
bool TreePlanner<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads) {
    this->setSamplingParams(start, goal);
    if (!setInitNode(start))
        return false;

    if (!m_validityChecker->check(goal)) {
        Logging::error("Goal Node in collision", this);
        return false;
    }

    std::vector<Vector<dim>> query = {goal};
    m_evaluator->setConfigs(query);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::info("Iteration: " + std::to_string(loopCount++), this);
        computeTree(numNodes, numThreads);
    }

    Logging::info("Planner has: " + std::to_string(m_graph->numNodes()) + " nodes", this);
    Logging::info("Planner has: " + std::to_string(m_graph->numEdges()) + " edges", this);

    return connectGoalNode(goal);
}

/*!
*  \brief      Compute path from start Node<dim> to goal Node<dim> with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2017-06-20
*/
template <unsigned int dim>
bool TreePlanner<dim>::computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, size_t numNodes, size_t numThreads) {
    //this->setSamplingParams(start, goal);
    if (!setInitNode(startConfig))
        return false;

    //std::vector<Vector<dim>> query = { goal };
    //m_evaluator->setConfigs(query);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::info("Iteration: " + std::to_string(loopCount++), this);
        computeTree(numNodes, numThreads);
    }

    Logging::info("Planner has: " + std::to_string(m_graph->numNodes()) + " nodes", this);
    Logging::info("Planner has: " + std::to_string(m_graph->numEdges()) + " edges", this);

    return true;
    //return connectGoalNode(goal);
}

/*!
*  \brief      Expands tree, if init Node is set.
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if valid
*  \date       2017-06-20
*/
template <unsigned int dim>
bool TreePlanner<dim>::expand(size_t numNodes, size_t numThreads) {
    return computeTree(numNodes, numThreads);
}

/*!
*  \brief      Set init Node of the TreePlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if valid
*  \date       2017-06-20
*/
template <unsigned int dim>
bool TreePlanner<dim>::setInitNode(const Vector<dim> start) {
    if (m_initNode) {
        if (start == m_initNode->getValues()) {
            Logging::info("Equal start node, tree will be expanded", this);
            return true;
        } else {
            Logging::info("New start node, new tree will be created", this);
            m_graph = std::make_shared<Graph<dim>>(m_graph->getSortCount(), m_graph->getNeighborFinder());
            m_graph->sortTree();
        }
    }

    if (!m_validityChecker->check(start)) {
        Logging::warning("Init Node could not be connected", this);
        return false;
    }

    m_initNode = std::make_shared<Node<dim>>(start);
    m_graph->addNode(m_initNode);
    return true;
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> TreePlanner<dim>::getPathNodes() {
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    if (!m_pathPlanned)
        return nodes;

    nodes.push_back(m_goalNode);
    for (std::shared_ptr<Node<dim>> temp = m_goalNode->getParentNode(); temp != nullptr; temp = temp->getParentNode())
        nodes.push_back(temp);

    std::reverse(nodes.begin(), nodes.end());

    Logging::info("Path has: " + std::to_string(nodes.size()) + " nodes", this);
    return nodes;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[in]  trajectoryStepSize of the result path
*  \param[out] configurations of the path
*  \date       2017-06-20
*/
template <unsigned int dim>
std::vector<Vector<dim>> TreePlanner<dim>::getPath(double posRes, double oriRes) {
    std::vector<Vector<dim>> path;
    if (!m_pathPlanned) {
        Logging::warning("Path is not complete", this);
        return path;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = getPathNodes();
    path = this->getPathFromNodes(nodes, posRes, oriRes);

    Logging::info("Path has: " + std::to_string(path.size()) + " points", this);
    return path;
}

/*!
*  \brief      Return the init Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] init Node
*  \date       2017-06-20
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> TreePlanner<dim>::getInitNode() const {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
*  \date       2017-06-20
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> TreePlanner<dim>::getGoalNode() const {
    return m_goalNode;
}

} /* namespace ippp */

#endif /* TREETPLANNER_HPP */
