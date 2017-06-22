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

#include <ippp/pathPlanner/Planner.hpp>
#include <ippp/pathPlanner/options/RRTOptions.hpp>

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

    virtual bool computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes, const unsigned int numThreads);
    virtual bool expand(const unsigned int numNodes, const unsigned int numThreads);
    virtual bool setInitNode(const Vector<dim> start);

    virtual bool computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads = 1) = 0;
    virtual bool connectGoalNode(const Vector<dim> goal) = 0;

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(const double trajectoryStepSize = 1);
    std::shared_ptr<Node<dim>> getInitNode() const;
    std::shared_ptr<Node<dim>> getGoalNode() const;

  protected:
    std::shared_ptr<Node<dim>> m_initNode = nullptr;
    std::shared_ptr<Node<dim>> m_goalNode = nullptr;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::m_environment;
};

/*!
*  \brief      Constructor of the class TreePlanner, base class of RRT and EST.
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  options
*  \param[in]  Graph
*  \param[in]  name
*  \date       2017-06-20
*/
template <unsigned int dim>
TreePlanner<dim>::TreePlanner(const std::string &name, const std::shared_ptr<Environment> &environment, const PlannerOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph)
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
bool TreePlanner<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes,
                           const unsigned int numThreads) {
    if (!setInitNode(start)) {
        return false;
    }
    if (m_collision->checkConfig(goal)) {
        Logging::error("Goal Node in collision", this);
        return false;
    }
    computeTree(numNodes, numThreads);

    return connectGoalNode(goal);
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
bool TreePlanner<dim>::expand(const unsigned int numNodes, const unsigned int numThreads) {
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
            m_graph = std::shared_ptr<Graph<dim>>(new Graph<dim>(m_graph->getSortCount(), m_graph->getNeighborFinder()));
            m_graph->sortTree();
        }
    }

    if (m_collision->checkConfig(start)) {
        Logging::warning("Init Node could not be connected", this);
        return false;
    }

    this->m_sampling->setOrigin(start);
    m_initNode = std::shared_ptr<Node<dim>>(new Node<dim>(start));
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
    if (!m_pathPlanned) {
        return nodes;
    }

    nodes.push_back(m_goalNode);
    for (std::shared_ptr<Node<dim>> temp = m_goalNode->getParentNode(); temp != nullptr; temp = temp->getParentNode()) {
        nodes.push_back(temp);
    }

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
std::vector<Vector<dim>> TreePlanner<dim>::getPath(const double trajectoryStepSize) {
    std::vector<Vector<dim>> path;
    if (!m_pathPlanned) {
        Logging::warning("Path is not complete", this);
        return path;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = getPathNodes();
    path = this->getPathFromNodes(nodes, trajectoryStepSize);

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
