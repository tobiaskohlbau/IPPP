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

#ifndef SRT_HPP
#define SRT_HPP

#include <mutex>

#include <ippp/planner/Planner.hpp>
#include <ippp/planner/options/SRTOptions.hpp>

namespace ippp {

/*!
* \brief   Class SRT "Sampling Based Roadmap of Trees"
* \author  Sascha Kaden
* \date    2017-04-02
*/
template <unsigned int dim>
class SRT : public Planner<dim> {
  public:
    SRT(const std::shared_ptr<Environment> &environment, const SRTOptions<dim> &options,
        const std::shared_ptr<Graph<dim>> &graph);

    bool computePath(Vector<dim> start, Vector<dim> goal, size_t numNodes, size_t numThreads);
    bool expand(size_t numNodes, size_t numThreads);

    void startSamplingPhase(size_t nbOfNodes, size_t nbOfThreads = 1);
    bool plannerPhase(std::vector<std::shared_ptr<Graph<dim>>> &trees);

    bool queryPath(Vector<dim> start, Vector<dim> goal);
    bool aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode);
    void expandNode(std::shared_ptr<Node<dim>> currentNode);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(double trajectoryStepSize = 1);

  protected:
    void samplingPhase(size_t nbOfNodes, size_t nbOfTrees);
    std::shared_ptr<Graph<dim>> computeTree(size_t nbOfNodes, const Vector<dim> &origin);

    size_t m_nbOfTrees;
    std::vector<std::shared_ptr<Graph<dim>>> m_treeGraphs;
    std::vector<std::shared_ptr<Node<dim>>> m_nodePath;
    std::vector<std::shared_ptr<Node<dim>>> m_openList, m_closedList;

    std::mutex m_mutex;

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
*  \brief      Standard constructor of the class SRT
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  prm options
*  \date       2017-04-03
*/
template <unsigned int dim>
SRT<dim>::SRT(const std::shared_ptr<Environment> &environment, const SRTOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph)
    : Planner<dim>("SRT", environment, options, graph) {
    m_nbOfTrees = options.getNbOfTrees();
}

/*!
*  \brief      Compute path from start Node to goal Node with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2017-04-03
*/
template <unsigned int dim>
bool SRT<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, const size_t numNodes, const size_t numThreads) {
    if (m_collision->checkConfig(start)) {
        Logging::error("Start Node in collision", this);
        return false;
    }
    if (m_collision->checkConfig(goal)) {
        Logging::error("Goal Node in collision", this);
        return false;
    }

    std::vector<Vector<dim>> query = {start, goal};
    m_evaluator->setQuery(query);

    while (!m_evaluator->evaluate())
        expand(numNodes, numThreads);

    return queryPath(start, goal);
}

/*!
*  \brief      Expands graph
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] error flag
*  \date       2017-04-03
*/
template <unsigned int dim>
bool SRT<dim>::expand(const size_t numNodes, const size_t numThreads) {
    startSamplingPhase(numNodes, numThreads);
    plannerPhase(m_treeGraphs);
    m_treeGraphs.clear();
    return true;
}

/*!
*  \brief      Sampling phase of the Pipppanner
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of threads
*  \date       2017-04-03
*/
template <unsigned int dim>
void SRT<dim>::startSamplingPhase(const size_t nbOfNodes, const size_t nbOfThreads) {
    if (nbOfThreads == 1) {
        samplingPhase(nbOfNodes, m_nbOfTrees);
    } else {
        size_t treeCount = (m_nbOfTrees / nbOfThreads) + 1;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&SRT::samplingPhase, this, nbOfNodes, treeCount));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Sampling thread function
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of trees to be sampled
*  \date       2017-04-03
*/
template <unsigned int dim>
void SRT<dim>::samplingPhase(const size_t nbOfNodes, const size_t nbOfTrees) {
    Vector<dim> sample;
    for (size_t i = 0; i < nbOfTrees; ++i) {
        do {
            sample = m_sampling->getSample();
        } while (util::empty<dim>(sample) || m_collision->checkConfig(sample));

        std::shared_ptr<Graph<dim>> graph = computeTree(nbOfNodes, sample);
        m_mutex.lock();
        m_treeGraphs.push_back(graph);
        m_mutex.unlock();
    }
}

/*!
*  \brief      Computes rrt* tree and returns computed the graph
*  \author     Sascha Kaden
*  \param[in]  number of Nodes
*  \param[in]  origin of tree
*  \date       2017-04-03
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> SRT<dim>::computeTree(const size_t nbOfNodes, const Vector<dim> &origin) {
    RRTOptions<dim> rrtOptions(30, m_collision, m_metric, m_evaluator, this->m_pathModifier, m_sampling, m_trajectory);
    RRT<dim> rrt(m_environment, rrtOptions);
    rrt.setInitNode(origin);
    rrt.expand(nbOfNodes, 1);
    return rrt.getGraph();
}

/*!
*  \brief      Local planning phase of the SRT.
*  \details    Add the nearest neighbors of a Node as childes.
*  \author     Sascha Kaden
*  \param[in]  vector of trees (Graphs)
*  \date       2017-04-03
*/
template <unsigned int dim>
bool SRT<dim>::plannerPhase(std::vector<std::shared_ptr<Graph<dim>>> &trees) {
    if (trees.empty()) {
        Logging::warning("Planning phase with empty tree list", this);
        return false;
    }

    std::sort(trees.begin(), trees.end());
    // if graph is empty, add first tree and remove tree from list
    if (m_graph->empty()) {
        m_graph->addNodeList(trees[0]->getNodes());
        trees[0]->preserveNodePtr();
        m_treeGraphs.erase(trees.begin());
    }

    // add all graphs from tree list to the main graph
    size_t connectionCount = 0;
    for (auto graph : trees) {
        for (size_t i = 0; i < 20; ++i) {
            int count = m_sampling->getRandomNumber() * graph->size();
            std::shared_ptr<Node<dim>> node = graph->getNode(count);
            std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(node);
            if (m_trajectory->checkTrajectory(node, nearestNode)) {
                node->addChild(nearestNode, m_metric->calcDist(node, nearestNode));
                nearestNode->addChild(node, m_metric->calcDist(nearestNode, node));
                m_graph->addNodeList(graph->getNodes());
                m_graph->sortTree();
                ++connectionCount;
                graph->preserveNodePtr();
                break;
            }
        }
    }

    bool result = false;
    if (connectionCount == trees.size())
        result = true;
    return result;
}

/*!
*  \brief      Search path between start and goal Node
*  \details    Uses internal the A* algorithm to find the best path. It saves the path Nodes internal.
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[out] result of query
*  \date       2017-04-03
*/
template <unsigned int dim>
bool SRT<dim>::queryPath(const Vector<dim> start, const Vector<dim> goal) {
    std::vector<std::shared_ptr<Graph<dim>>> trees;
    trees.push_back(computeTree(100, start));
    trees.push_back(computeTree(100, goal));
    if (!plannerPhase(trees)) {
        Logging::info("Start or goal Node could not be connected", this);
        return false;
    }

    std::shared_ptr<Node<dim>> sourceNode = trees[0]->getNode(0);
    std::shared_ptr<Node<dim>> targetNode = trees[1]->getNode(0);
    bool pathPlanned = aStar(sourceNode, targetNode);

    if (pathPlanned) {
        Logging::info("Path could be planned", this);
        m_nodePath.push_back(std::shared_ptr<Node<dim>>(new Node<dim>(goal)));
        std::shared_ptr<Node<dim>> temp = targetNode;
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
*  \brief      A* algorithm to find best path
*  \author     Sascha Kaden
*  \param[in]  source Node (start)
*  \param[in]  target Node (goal)
*  \param[out] result of algorithm
*  \date       2017-04-03
*/
template <unsigned int dim>
bool SRT<dim>::aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode) {
    m_graph->clearQueryParents();
    m_closedList.clear();
    m_openList.clear();

    std::vector<std::shared_ptr<Edge<dim>>> edges = sourceNode->getChildEdges();
    for (size_t i = 0; i < edges.size(); ++i) {
        edges[i]->getTarget()->setCost(edges[i]->getCost());
        edges[i]->getTarget()->setQueryParent(sourceNode, m_metric->calcEdgeCost(edges[i]->getTarget(), sourceNode));
        m_openList.push_back(edges[i]->getTarget());
    }
    m_closedList.push_back(sourceNode);

    int count = 0;
    std::shared_ptr<Node<dim>> currentNode;
    while (!m_openList.empty()) {
        currentNode = util::removeMinFromList(m_openList);

        if (currentNode == targetNode)
            return true;

        m_closedList.push_back(currentNode);
        ++count;

        expandNode(currentNode);
    }
    return false;
}

/*!
*  \brief      Expands the openList of the A* algorithm from the childes of the passed Node
*  \author     Sascha Kaden
*  \param[in]  current node
*  \date       2017-04-03
*/
template <unsigned int dim>
void SRT<dim>::expandNode(std::shared_ptr<Node<dim>> currentNode) {
    double dist, edgeCost;
    for (auto successor : currentNode->getChildNodes()) {
        if (util::contains(m_closedList, successor))
            continue;

        edgeCost = m_metric->calcEdgeCost(currentNode, successor);
        dist = currentNode->getCost() + edgeCost;

        if (util::contains(m_openList, successor) && dist >= successor->getCost())
            continue;

        successor->setQueryParent(currentNode, edgeCost);
        successor->setCost(dist);
        if (!util::contains(m_openList, successor)) {
            m_openList.push_back(successor);
        }
    }
    if (currentNode->getParentNode() != nullptr) {
        auto successor = currentNode->getParentNode();
        if (util::contains(m_closedList, successor))
            return;

        edgeCost = m_metric->calcEdgeCost(currentNode, successor);
        dist = currentNode->getCost() + edgeCost;

        if (util::contains(m_openList, successor) && dist >= successor->getCost())
            return;

        successor->setQueryParent(currentNode, edgeCost);
        successor->setCost(dist);
        if (!util::contains(m_openList, successor))
            m_openList.push_back(successor);
    }
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2017-04-03
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> SRT<dim>::getPathNodes() {
    return m_nodePath;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[in]  trajectory step size
*  \param[out] configurations of the path
*  \date       2017-04-03
*/
template <unsigned int dim>
std::vector<Vector<dim>> SRT<dim>::getPath(const double trajectoryStepSize) {
    return this->getPathFromNodes(m_nodePath, trajectoryStepSize);
}

} /* namespace ippp */

#endif    // SRT_HPP
