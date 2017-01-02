//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef PRMPLANNER_H_
#define PRMPLANNER_H_

#include <pathPlanner/Planner.hpp>
#include <pathPlanner/options/PRMOptions.h>

namespace rmpl {

/*!
* \brief   Class PRMPlanner
* \author  Sascha Kaden
* \date    2016-08-09
*/
template <unsigned int dim>
class PRMPlanner : public Planner<dim> {
  public:
    PRMPlanner(const std::shared_ptr<RobotBase<dim>> &robot, const PRMOptions &options);

    bool computePath(Vector<dim> start, Vector<dim> goal, unsigned int numNodes, unsigned int numThreads);

    void startSamplingPhase(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    void startPlannerPhase(unsigned int nbOfThreads = 1);

    bool queryPath(Vector<dim> start, Vector<dim> goal);
    bool aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode);
    void expandNode(std::shared_ptr<Node<dim>> currentNode);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(float trajectoryStepSize, bool smoothing = true);

  protected:
    void samplingPhase(unsigned int nbOfNodes);
    void plannerPhase(unsigned int startNodeIndex, unsigned int endNodeIndex);
    std::shared_ptr<Node<dim>> connectNode(Vector<dim> &node);

    float m_rangeSize;
    std::vector<std::shared_ptr<Node<dim>>> m_nodePath;
    std::vector<std::shared_ptr<Node<dim>>> m_openList, m_closedList;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_planner;
    using Planner<dim>::m_robot;
    using Planner<dim>::m_sampler;
};

/*!
*  \brief      Standard constructor of the class PRMPlanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  rangeSize for the nearest neighbor search from the local planner
*  \param[in]  trajectoryStepSize
*  \param[in]  TrajectoryMethod
*  \param[in]  SamplerMethod
*  \date       2016-08-09
*/
template <unsigned int dim>
PRMPlanner<dim>::PRMPlanner(const std::shared_ptr<RobotBase<dim>> &robot, const PRMOptions &options)
    : Planner<dim>("PRMPlanner", robot, options) {
    m_rangeSize = options.getRangeSize();
}

/*!
*  \brief      Compute path from start Node<dim> to goal Node<dim> with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2016-05-27
*/
template <unsigned int dim>
bool PRMPlanner<dim>::computePath(Vector<dim> start, Vector<dim> goal, unsigned int numNodes, unsigned int numThreads) {
    startSamplingPhase(numNodes, numThreads);
    startPlannerPhase(numThreads);

    return queryPath(start, goal);
}

/*!
*  \brief      Sampling phase of the PRMPlanner
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRMPlanner<dim>::startSamplingPhase(unsigned int nbOfNodes, unsigned int nbOfThreads) {
    if (nbOfThreads == 1) {
        samplingPhase(nbOfNodes);
    } else {
        nbOfNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (int i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&PRMPlanner::samplingPhase, this, nbOfNodes));

        for (int i = 0; i < nbOfThreads; ++i)
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
void PRMPlanner<dim>::samplingPhase(unsigned int nbOfNodes) {
    for (int i = 0; i < nbOfNodes; ++i) {
        Vector<dim> sample = m_sampler->getSample();
        if (!m_collision->controlVec(sample)) {
            m_graph->addNode(std::shared_ptr<Node<dim>>(new Node<dim>(sample)));
        }
    }
}

/*!
*  \brief      Local planning phase of the PRMPlanner.
*  \details    Add the nearest neighbors of a Node<dim> as childs.
*  \author     Sascha Kaden
*  \param[in]  number threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRMPlanner<dim>::startPlannerPhase(unsigned int nbOfThreads) {
    unsigned int nodeCount = m_graph->size();
    if (nbOfThreads == 1) {
        plannerPhase(0, nodeCount);
    } else {
        unsigned int threadAmount = nodeCount / nbOfThreads;
        std::vector<std::thread> threads;

        for (int i = 0; i < nbOfThreads; ++i) {
            threads.push_back(std::thread(&PRMPlanner::plannerPhase, this, i * threadAmount, (i + 1) * threadAmount));
        }

        for (int i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Local planning thread function
*  \details    Searchs the nearest neighbors between the given indexes and adds them as childs
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  end index
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRMPlanner<dim>::plannerPhase(unsigned int startNodeIndex, unsigned int endNodeIndex) {
    if (startNodeIndex > endNodeIndex) {
        Logging::error("Start index is larger than end index", this);
        return;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = m_graph->getNodes();
    if (endNodeIndex > nodes.size()) {
        Logging::error("End index is larger than Node size", this);
        return;
    }

    for (auto node = nodes.begin() + startNodeIndex;
         node != nodes.begin() + endNodeIndex; ++node) {
        std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(*node, m_rangeSize);
        for (auto &nearNode : nearNodes) {
            if (m_planner->controlTrajectory((*node)->getValues(), nearNode->getValues()))
                (*node)->addChild(nearNode);
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
bool PRMPlanner<dim>::queryPath(Vector<dim> start, Vector<dim> goal) {
    std::shared_ptr<Node<dim>> sourceNode = connectNode(start);
    std::shared_ptr<Node<dim>> targetNode = connectNode(goal);
    if (sourceNode == nullptr || targetNode == nullptr) {
        Logging::info("Start or goal Node<dim> could not be connected", this);
        return false;
    }

    bool pathPlanned = aStar(sourceNode, targetNode);

    if (pathPlanned) {
        Logging::info("Path could be planned", this);
        m_nodePath.push_back(std::shared_ptr<Node<dim>>(new Node<dim>(goal)));
        std::shared_ptr<Node<dim>> temp = targetNode;
        int count = 0;
        while (temp != nullptr) {
            ++count;
            m_nodePath.push_back(temp);
            temp = temp->getParentNode();
        }
        m_nodePath.push_back(std::shared_ptr<Node<dim>>(new Node<dim>(start)));
        return true;
    } else {
        Logging::info("Path could NOT be planned", this);
        return false;
    }
}

/*!
*  \brief      Try to find nearest Node<dim> of the graph to the passed Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[in]  nearest Node
*  \date       2016-08-09
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> PRMPlanner<dim>::connectNode(Vector<dim> &vec) {
    std::vector<std::shared_ptr<Node<dim>>> nearNodes =
        m_graph->getNearNodes(std::shared_ptr<Node<dim>>(new Node<dim>(vec)), m_rangeSize * 3);
    float dist = std::numeric_limits<float>::max();
    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (m_planner->controlTrajectory(vec, *nearNodes[i]) && (vec - nearNodes[i]->getValues()).norm() < dist) {
            dist = (vec - nearNodes[i]->getValues()).norm();
            nearestNode = nearNodes[i];
        }
    }

    return nearestNode;
}

/*!
*  \brief      A* algorithm to find best path
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  source Node<dim> (start)
*  \param[in]  target Node<dim> (goal)
*  \param[out] result of algorithm
*  \date       2016-08-09
*/
template <unsigned int dim>
bool PRMPlanner<dim>::aStar(std::shared_ptr<Node<dim>> sourceNode, std::shared_ptr<Node<dim>> targetNode) {
    m_closedList.clear();
    m_openList.clear();

    std::vector<std::shared_ptr<Edge<dim>>> edges = sourceNode->getChildEdges();
    for (int i = 0; i < edges.size(); ++i) {
        edges[i]->getTarget()->setCost(edges[i]->getCost());
        edges[i]->getTarget()->setParent(sourceNode);
        m_openList.push_back(edges[i]->getTarget());
    }
    m_closedList.push_back(sourceNode);

    int count = 0;
    std::shared_ptr<Node<dim>> currentNode;
    while (!m_openList.empty()) {
        currentNode = utilList::removeMinFromList(m_openList);

        if (currentNode == targetNode) {
            std::cout << "closed list has: " << count << std::endl;
            return true;
        }

        m_closedList.push_back(currentNode);
        ++count;

        expandNode(currentNode);
    }
    std::cout << "closed list has: " << count << std::endl;

    return false;
}

/*!
*  \brief      Expands the openList of the A* algorithm from the childs of the passed Node
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  end index
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRMPlanner<dim>::expandNode(std::shared_ptr<Node<dim>> currentNode) {
    float dist;
    for (auto successor : currentNode->getChildNodes()) {
        if (utilList::contains(m_closedList, successor))
            continue;

        dist = currentNode->getCost() + Heuristic<dim>::calcEdgeCost(currentNode, successor);

        if (utilList::contains(m_openList, successor) && dist >= successor->getCost())
            continue;

        successor->setParent(currentNode);
        successor->setCost(dist);
        if (!utilList::contains(m_openList, successor))
            m_openList.push_back(successor);
    }
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> PRMPlanner<dim>::getPathNodes() {
    return m_nodePath;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[out] vecs of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<Vector<dim>> PRMPlanner<dim>::getPath(float trajectoryStepSize, bool smoothing) {
    return getPathFromNodes(m_nodePath, trajectoryStepSize, smoothing);
}

} /* namespace rmpl */

#endif    // PRMPLANNER_H_
