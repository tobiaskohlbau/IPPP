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

#ifndef PRM_HPP
#define PRM_HPP

#include <ippp/pathPlanner/Planner.hpp>
#include <ippp/pathPlanner/options/PRMOptions.hpp>

namespace ippp {

/*!
* \brief   Class PRM
* \author  Sascha Kaden
* \date    2016-08-09
*/
template <unsigned int dim>
class PRM : public Planner<dim> {
  public:
    PRM(const std::shared_ptr<Environment> &environment, const PRMOptions<dim> &options,
        const std::shared_ptr<Graph<dim>> &graph);

    bool computePath(const Vector<dim> start, const Vector<dim> goal, const size_t numNodes, const size_t numThreads);
    bool expand(const size_t numNodes, const size_t numThreads);

    void startSamplingPhase(const size_t nbOfNodes, const size_t nbOfThreads = 1);
    void startPlannerPhase(const size_t nbOfThreads = 1);

    bool queryPath(const Vector<dim> start, const Vector<dim> goal);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(const double posRes = 1, const double oriRes = 0.1);

  protected:
    void samplingPhase(const size_t nbOfNodes);
    void plannerPhase(const size_t startNodeIndex, const size_t endNodeIndex);
    std::shared_ptr<Node<dim>> connectNode(const Vector<dim> &config);

    double m_rangeSize;
    std::vector<std::shared_ptr<Node<dim>>> m_nodePath;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_evaluator;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_metric;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::m_sampling;
};

/*!
*  \brief      Standard constructor of the class PRM
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  PRMOptions
*  \param[in]  Graph
*  \date       2016-08-09
*/
template <unsigned int dim>
PRM<dim>::PRM(const std::shared_ptr<Environment> &environment, const PRMOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph)
    : Planner<dim>("PRM", environment, options, graph) {
    m_rangeSize = options.getRangeSize();
}

/*!
*  \brief      Compute path from start Node to goal Node with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2016-05-27
*/
template <unsigned int dim>
bool PRM<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, const size_t numNodes, const size_t numThreads) {
    std::vector<Vector<dim>> query = {start, goal};
    m_evaluator->setQuery(query);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        expand(numNodes, numThreads);
    }

    Logging::debug("Planner has: " + std::to_string(m_graph->nodeSize()) + " nodes", this);
    Logging::debug("Planner has: " + std::to_string(m_graph->edgeSize()) + " edges", this);

    return queryPath(start, goal);
}

/*!
*  \brief      Expands graph
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] error flag
*  \date       2017-03-01
*/
template <unsigned int dim>
bool PRM<dim>::expand(const size_t numNodes, const size_t numThreads) {
    startSamplingPhase(numNodes, numThreads);
    m_graph->sortTree();
    startPlannerPhase(numThreads);
    return true;
}

/*!
*  \brief      Sampling phase of the PRM
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startSamplingPhase(const size_t nbOfNodes, const size_t nbOfThreads) {
    size_t countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        samplingPhase(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&PRM::samplingPhase, this, countNodes));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Sampling thread function
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::samplingPhase(const size_t nbOfNodes) {
    Vector<dim> sample;
    for (size_t i = 0; i < nbOfNodes; ++i) {
        sample = m_sampling->getSample();
        if (util::empty<dim>(sample))
            continue;

        if (!m_collision->checkConfig(sample)) {
            m_graph->addNode(std::shared_ptr<Node<dim>>(new Node<dim>(sample)));
        }
    }
}

/*!
*  \brief      Local planning phase of the PRM.
*  \details    Add the nearest neighbors of a Node as childes.
*  \author     Sascha Kaden
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startPlannerPhase(const size_t nbOfThreads) {
    size_t nodeCount = m_graph->size();
    if (nbOfThreads == 1) {
        plannerPhase(0, nodeCount);
    } else {
        size_t threadAmount = nodeCount / nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&PRM::plannerPhase, this, i * threadAmount, (i + 1) * threadAmount));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Local planning thread function
*  \details    Searches the nearest neighbors between the given indexes and adds them as childes
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  end index
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::plannerPhase(const size_t startNodeIndex, const size_t endNodeIndex) {
    if (startNodeIndex > endNodeIndex) {
        Logging::error("Start index is larger than end index", this);
        return;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = m_graph->getNodes();
    if (endNodeIndex > nodes.size()) {
        Logging::error("End index is larger than Node size", this);
        return;
    }

    for (auto node = nodes.begin() + startNodeIndex; node != nodes.begin() + endNodeIndex; ++node) {
        std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(*node, m_rangeSize);
        for (auto &nearNode : nearNodes) {
            if ((*node)->isChild(nearNode) || (*node)->isInvalidChild(nearNode))
                continue;

            if (m_trajectory->checkTrajectory((*node)->getValues(), nearNode->getValues()))
                (*node)->addChild(nearNode, m_metric->calcDist(nearNode, (*node)));
            else
                (*node)->addInvalidChild(nearNode);
        }
    }
}

/*!
*  \brief      Searches a between start and goal Node
*  \details    Uses internal the A* algorithm to find the best path. It saves the path Nodes internal.
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[out] result of query
*  \date       2016-08-09
*/
template <unsigned int dim>
bool PRM<dim>::queryPath(const Vector<dim> start, const Vector<dim> goal) {
    std::shared_ptr<Node<dim>> sourceNode = connectNode(start);
    std::shared_ptr<Node<dim>> goalNode = connectNode(goal);
    if (sourceNode == nullptr || goalNode == nullptr) {
        Logging::info("Start or goal Node could not be connected", this);
        return false;
    }

    m_graph->clearQueryParents();
    bool pathPlanned = util::aStar<dim>(sourceNode, goalNode, m_metric);

    if (pathPlanned) {
        Logging::info("Path could be planned", this);
        m_nodePath.push_back(std::shared_ptr<Node<dim>>(new Node<dim>(goal)));
        std::shared_ptr<Node<dim>> temp = goalNode;
        int count = 0;
        while (temp != nullptr) {
            ++count;
            m_nodePath.push_back(temp);
            temp = temp->getQueryParentNode();
        }
        m_nodePath.push_back(std::shared_ptr<Node<dim>>(new Node<dim>(start)));
        return true;
    } else {
        Logging::info("Path could NOT be planned", this);
        return false;
    }
}

/*!
*  \brief      Try to find nearest Node of the graph to the passed Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[in]  nearest Node
*  \date       2016-08-09
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> PRM<dim>::connectNode(const Vector<dim> &config) {
    // check that the graph contains the node and if yes return it
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNode(config);
    if (nearestNode)
        return nearestNode;

    nearestNode = util::getNearestValidNode<dim>(config, m_graph, m_trajectory, m_metric, m_rangeSize);
    // create new Node with connection to the nearest Node, if a valid path exists
    if (nearestNode) {
        std::shared_ptr<Node<dim>> node(new Node<dim>(config));
        node->setParent(nearestNode, m_metric->calcDist(nearestNode, node));
        nearestNode->addChild(node, m_metric->calcDist(nearestNode, node));
        return node;
    } else {
        return nullptr;
    }
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> PRM<dim>::getPathNodes() {
    return m_nodePath;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[in]  trajectory step size
*  \param[out] configurations of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<Vector<dim>> PRM<dim>::getPath(const double posRes, const double oriRes) {
    return this->getPathFromNodes(m_nodePath, posRes, oriRes);
}

} /* namespace ippp */

#endif    // PRM_HPP
