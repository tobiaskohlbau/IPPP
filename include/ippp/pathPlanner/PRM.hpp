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
* \brief   Class Pipppanner
* \author  Sascha Kaden
* \date    2016-08-09
*/
template <unsigned int dim>
class PRM : public Planner<dim> {
  public:
    PRM(const std::shared_ptr<Environment> &environment, const PRMOptions<dim> &options,
        const std::shared_ptr<Graph<dim>> &graph);

    bool computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes, const unsigned int numThreads);
    bool expand(const unsigned int numNodes, const unsigned int numThreads);

    void startSamplingPhase(const unsigned int nbOfNodes, const unsigned int nbOfThreads = 1);
    void startPlannerPhase(const unsigned int nbOfThreads = 1);

    bool queryPath(const Vector<dim> start, const Vector<dim> goal);
    bool aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode);
    void expandNode(std::shared_ptr<Node<dim>> currentNode);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(const double trajectoryStepSize = 1);

  protected:
    void samplingPhase(const unsigned int nbOfNodes);
    void plannerPhase(const unsigned int startNodeIndex, const unsigned int endNodeIndex);
    std::shared_ptr<Node<dim>> connectNode(const Vector<dim> &node);

    double m_rangeSize;
    std::vector<std::shared_ptr<Node<dim>>> m_nodePath;
    std::vector<std::shared_ptr<Node<dim>>> m_openList, m_closedList;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::m_environment;
    using Planner<dim>::m_sampling;
    using Planner<dim>::m_metric;
};

/*!
*  \brief      Standard constructor of the class Pipppanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  prm options
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
bool PRM<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes,
                           const unsigned int numThreads) {
    expand(numNodes, numThreads);

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
bool PRM<dim>::expand(const unsigned int numNodes, const unsigned int numThreads) {
    startSamplingPhase(numNodes, numThreads);
    m_graph->sortTree();
    startPlannerPhase(numThreads);
    return true;
}

/*!
*  \brief      Sampling phase of the Pipppanner
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startSamplingPhase(const unsigned int nbOfNodes, const unsigned int nbOfThreads) {
    unsigned int countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        samplingPhase(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads.push_back(std::thread(&PRM::samplingPhase, this, countNodes));
        }

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads[i].join();
        }
    }
}

/*!
*  \brief      Sampling thread function
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::samplingPhase(const unsigned int nbOfNodes) {
    Vector<dim> sample;
    for (unsigned int i = 0; i < nbOfNodes; ++i) {
        sample = m_sampling->getSample();
        if (util::empty<dim>(sample))
            continue;

        if (!m_collision->checkConfig(sample)) {
            m_graph->addNode(std::shared_ptr<Node<dim>>(new Node<dim>(sample)));
        }
    }
}

/*!
*  \brief      Local planning phase of the Pipppanner.
*  \details    Add the nearest neighbors of a Node as childes.
*  \author     Sascha Kaden
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startPlannerPhase(const unsigned int nbOfThreads) {
    unsigned int nodeCount = m_graph->size();
    if (nbOfThreads == 1) {
        plannerPhase(0, nodeCount);
    } else {
        unsigned int threadAmount = nodeCount / nbOfThreads;
        std::vector<std::thread> threads;

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads.push_back(std::thread(&PRM::plannerPhase, this, i * threadAmount, (i + 1) * threadAmount));
        }

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads[i].join();
        }
    }
}

/*!
*  \brief      Local planning thread function
*  \details    Searchs the nearest neighbors between the given indexes and adds them as childes
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  end index
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::plannerPhase(const unsigned int startNodeIndex, const unsigned int endNodeIndex) {
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
            if (m_trajectory->checkTrajectory((*node)->getValues(), nearNode->getValues())) {
                (*node)->addChild(nearNode, m_metric->calcDist(nearNode, (*node)));
            }
        }
    }
}

/*!
*  \brief      Searchs a between start and goal Node
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

    bool pathPlanned = aStar(sourceNode, goalNode);

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
std::shared_ptr<Node<dim>> PRM<dim>::connectNode(const Vector<dim> &vec) {
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(vec, m_rangeSize * 3);
    double dist = std::numeric_limits<double>::max();
    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (m_trajectory->checkTrajectory(vec, *nearNodes[i]) && m_metric->calcDist(vec, nearNodes[i]->getValues()) < dist) {
            dist = m_metric->calcDist(vec, nearNodes[i]->getValues());
            nearestNode = nearNodes[i];
        }
    }

    return nearestNode;
}

/*!
*  \brief      A* algorithm to find best path
*  \author     Sascha Kaden
*  \param[in]  source Node (start)
*  \param[in]  goal Node (goal)
*  \param[out] result of algorithm
*  \date       2016-08-09
*/
template <unsigned int dim>
bool PRM<dim>::aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode) {
    m_graph->clearQueryParents();
    m_closedList.clear();
    m_openList.clear();

    std::vector<std::shared_ptr<Edge<dim>>> edges = sourceNode->getChildEdges();
    for (int i = 0; i < edges.size(); ++i) {
        edges[i]->getTarget()->setCost(edges[i]->getCost());
        edges[i]->getTarget()->setQueryParent(sourceNode, m_metric->calcDist(edges[i]->getTarget(), sourceNode));
        m_openList.push_back(edges[i]->getTarget());
    }
    m_closedList.push_back(sourceNode);

    int count = 0;
    std::shared_ptr<Node<dim>> currentNode;
    while (!m_openList.empty()) {
        currentNode = util::removeMinFromList(m_openList);

        if (currentNode == targetNode) {
            return true;
        }
        m_closedList.push_back(currentNode);
        ++count;

        expandNode(currentNode);
    }
    return false;
}

/*!
*  \brief      Expands the openList of the A* algorithm from the childes of the passed Node
*  \author     Sascha Kaden
*  \param[in]  current ndoe
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::expandNode(std::shared_ptr<Node<dim>> currentNode) {
    double dist, edgeCost;
    for (auto successor : currentNode->getChildNodes()) {
        if (util::contains(m_closedList, successor)) {
            continue;
        }
        edgeCost = m_metric->calcDist(currentNode, successor);
        dist = currentNode->getCost() + edgeCost;

        if (util::contains(m_openList, successor) && dist >= successor->getCost()) {
            continue;
        }
        successor->setQueryParent(currentNode, edgeCost);
        successor->setCost(dist);
        if (!util::contains(m_openList, successor)) {
            m_openList.push_back(successor);
        }
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
std::vector<Vector<dim>> PRM<dim>::getPath(const double trajectoryStepSize) {
    return this->getPathFromNodes(m_nodePath, trajectoryStepSize);
}

} /* namespace ippp */

#endif    // PRM_HPP