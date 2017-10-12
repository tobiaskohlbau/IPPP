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

#ifndef EST_HPP
#define EST_HPP

#include <mutex>

#include <ippp/pathPlanner/TreePlanner.hpp>

namespace ippp {

/*!
* \brief   Super class of the EST
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class EST : public TreePlanner<dim> {
  public:
    EST(const std::shared_ptr<Environment> &environment, const PlannerOptions<dim> &options,
        const std::shared_ptr<Graph<dim>> &graph);

    bool computeTree(size_t nbOfNodes, size_t nbOfThreads = 1);
    bool connectGoalNode(const Vector<dim> goal);

  protected:
    void computeTreeThread(size_t nbOfNodes);

    // variables
    std::mutex m_mutex;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_metric;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_sampling;
    using Planner<dim>::m_trajectory;
    using TreePlanner<dim>::m_initNode;
    using TreePlanner<dim>::m_goalNode;
};

/*!
*  \brief      Constructor of the class EST
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  options
*  \param[in]  Graph
*  \param[in]  name
*  \date       2017-06-20
*/
template <unsigned int dim>
EST<dim>::EST(const std::shared_ptr<Environment> &environment, const PlannerOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph)
    : TreePlanner<dim>("EST", environment, options, graph) {
}

/*!
*  \brief      Compute tree of the EST
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] return check of the constraints
*  \date       2017-06-20
*/
template <unsigned int dim>
bool EST<dim>::computeTree(const size_t nbOfNodes, const size_t nbOfThreads) {
    if (m_initNode == nullptr) {
        Logging::error("Init Node is not connected", this);
        return false;
    }

    size_t countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        computeTreeThread(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&EST::computeTreeThread, this, countNodes));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }

    return true;
}

/*!
*  \brief      Compute tree of the EST, threaded function
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \date       2017-06-20
*/
template <unsigned int dim>
void EST<dim>::computeTreeThread(const size_t nbOfNodes) {
    Vector<dim> sample;
    for (size_t i = 0; i < nbOfNodes; ++i) {
        // choose random node of the graph
        auto randNode = m_graph->getNode(m_sampling->getRandomNumber() * m_graph->size());
        sample = m_sampling->getSample(randNode->getValues());
        if (util::empty<dim>(sample) || m_collision->checkConfig(sample))
            continue;

        if (!m_trajectory->checkTrajectory(randNode->getValues(), sample))
            continue;

        std::shared_ptr<Node<dim>> newNode(new Node<dim>(sample));
        newNode->setParent(randNode, m_metric->calcDist(sample, randNode->getValues()));
        m_mutex.lock();
        randNode->addChild(newNode, m_metric->calcDist(randNode->getValues(), sample));
        m_mutex.unlock();

        m_graph->addNode(newNode);
    }
}

/*!
*  \brief      Connects goal Node to the tree, if connection is valid
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was valid
*  \date       2017-06-20
*/
template <unsigned int dim>
bool EST<dim>::connectGoalNode(Vector<dim> goal) {
    if (m_collision->checkConfig(goal)) {
        Logging::warning("Goal Node in collision", this);
        return false;
    }

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    // Todo: add option for the connection distance of the goal node
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(goalNode, 100);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (auto node : nearNodes) {
        if (m_trajectory->checkTrajectory(goal, node->getValues())) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode, this->m_metric->calcDist(nearestNode, goalNode));
        m_goalNode = goalNode;
        m_graph->addNode(goalNode);
        // Logging::info("Goal Node<dim> is connected", this);
        m_pathPlanned = true;
        Logging::info("Goal is connected", this);
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}

} /* namespace ippp */

#endif /* EST_HPP */
