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

#include <pathPlanner/PRMPlanner.h>

#include <core/utility/Logging.h>
#include <thread>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Standard constructor of the class PRMPlanner
*  \author     Sascha Kaden
*  \param[in]  robot
*  \param[in]  rangeSize for the nearest neighbor search from the local planner
*  \param[in]  trajectoryStepSize
*  \param[in]  TrajectoryMethod
*  \param[in]  SamplingMethod
*  \date       2016-08-09
*/
PRMPlanner::PRMPlanner(const shared_ptr<RobotBase> &robot, const PRMOptions &options) : Planner("PRMPlanner", robot, options) {
    m_rangeSize = options.getRangeSize();
}

bool PRMPlanner::computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads) {
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
void PRMPlanner::startSamplingPhase(unsigned int nbOfNodes, unsigned int nbOfThreads) {
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
void PRMPlanner::samplingPhase(unsigned int nbOfNodes) {
    for (int i = 0; i < nbOfNodes; ++i) {
        Eigen::VectorXf sample = m_sampler->getSample();
        if (!m_collision->controlVec(sample)) {
            m_graph->addNode(shared_ptr<Node>(new Node(sample)));
        }
    }
}

/*!
*  \brief      Local planning phase of the PRMPlanner.
*  \details    Add the nearest neighbors of a Node as childs.
*  \author     Sascha Kaden
*  \param[in]  number threads
*  \date       2016-08-09
*/
void PRMPlanner::startPlannerPhase(unsigned int nbOfThreads) {
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
void PRMPlanner::plannerPhase(unsigned int startNodeIndex, unsigned int endNodeIndex) {
    if (startNodeIndex > endNodeIndex) {
        Logging::error("Start index is larger than end index", this);
        return;
    }

    std::vector<shared_ptr<Node>> nodes = m_graph->getNodes();
    if (endNodeIndex > nodes.size()) {
        Logging::error("End index is larger than node size", this);
        return;
    }

    for (std::vector<shared_ptr<Node>>::iterator node = nodes.begin() + startNodeIndex; node != nodes.begin() + endNodeIndex;
         ++node) {
        std::vector<shared_ptr<Node>> nearNodes = m_graph->getNearNodes(*node, m_rangeSize);
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
bool PRMPlanner::queryPath(Eigen::VectorXf start, Eigen::VectorXf goal) {
    if (empty(start) || empty(goal)) {
        Logging::error("Start or goal node is empty", this);
        return false;
    }

    shared_ptr<Node> sourceNode = connectNode(start);
    shared_ptr<Node> targetNode = connectNode(goal);
    if (sourceNode == nullptr || targetNode == nullptr) {
        Logging::info("Start or goal Node could not be connected", this);
        return false;
    }

    bool pathPlanned = aStar(sourceNode, targetNode);

    if (pathPlanned) {
        Logging::info("Path could be planned", this);
        m_nodePath.push_back(shared_ptr<Node>(new Node(goal)));
        shared_ptr<Node> temp = targetNode;
        int count = 0;
        while (temp != nullptr) {
            ++count;
            m_nodePath.push_back(temp);
            temp = temp->getParentNode();
        }
        m_nodePath.push_back(shared_ptr<Node>(new Node(start)));
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
shared_ptr<Node> PRMPlanner::connectNode(Eigen::VectorXf &vec) {
    std::vector<shared_ptr<Node>> nearNodes = m_graph->getNearNodes(shared_ptr<Node>(new Node(vec)), m_rangeSize * 3);
    float dist = std::numeric_limits<float>::max();
    shared_ptr<Node> nearestNode = nullptr;
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
*  \param[in]  source Node (start)
*  \param[in]  target Node (goal)
*  \param[out] result of algorithm
*  \date       2016-08-09
*/
bool PRMPlanner::aStar(shared_ptr<Node> sourceNode, shared_ptr<Node> targetNode) {
    m_closedList.clear();
    m_openList.clear();

    std::vector<shared_ptr<Edge>> edges = sourceNode->getChildEdges();
    for (int i = 0; i < edges.size(); ++i) {
        edges[i]->getTarget()->setCost(edges[i]->getCost());
        edges[i]->getTarget()->setParent(sourceNode);
        m_openList.push_back(edges[i]->getTarget());
    }
    m_closedList.push_back(sourceNode);

    int count = 0;
    shared_ptr<Node> currentNode;
    while (!m_openList.empty()) {
        currentNode = utility::removeMinFromList(m_openList);

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
void PRMPlanner::expandNode(shared_ptr<Node> currentNode) {
    for (auto successor : currentNode->getChildNodes()) {
        if (utility::contains(m_closedList, successor))
            continue;

        float dist = currentNode->getCost() + currentNode->getDist(successor);

        if (utility::contains(m_openList, successor) && dist >= successor->getCost())
            continue;

        successor->setParent(currentNode);
        successor->setCost(dist);
        if (utility::contains(m_openList, successor)) {
            // nothing to do
        } else {
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
std::vector<shared_ptr<Node>> PRMPlanner::getPathNodes() {
    return m_nodePath;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[out] vecs of the path
*  \date       2016-05-31
*/
std::vector<Eigen::VectorXf> PRMPlanner::getPath(float trajectoryStepSize, bool smoothing) {
    return getPathFromNodes(m_nodePath, trajectoryStepSize, smoothing);
}

} /* namespace rmpl */