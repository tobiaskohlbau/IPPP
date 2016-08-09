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

#include <thread>

using namespace rmpl;
using std::shared_ptr;

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
PRMPlanner::PRMPlanner(const shared_ptr<RobotBase> &robot, float rangeSize, float trajectoryStepSize, TrajectoryMethod trajectory,
                       SamplingMethod sampling)
    : Planner("PRMPlanner", robot, trajectoryStepSize, trajectory, sampling) {
    m_rangeSize = rangeSize;
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
    unsigned int dim = this->m_robot->getDim();
    for (int i = 0; i < nbOfNodes; ++i) {
        Vec<float> sample = this->m_sampler->getSample(dim, i, nbOfNodes);
        if (!this->m_collision->controlCollision(sample)) {
            this->m_graph->addNode(shared_ptr<Node>(new Node(sample)));
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
    unsigned int nodeCount = this->m_graph->size();
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
        sendMessage("Start index is larger than end index", Message::error);
        return;
    }

    std::vector<shared_ptr<Node>> nodes = this->m_graph->getNodes();
    if (endNodeIndex > nodes.size()) {
        sendMessage("End index is larger than node size", Message::error);
        return;
    }

    for (std::vector<shared_ptr<Node>>::iterator node = nodes.begin() + startNodeIndex; node != nodes.begin() + endNodeIndex;
         ++node) {
        std::vector<shared_ptr<Node>> nearNodes = this->m_graph->getNearNodes(*node, m_rangeSize);
        for (auto &nearNode : nearNodes) {
            if (this->m_planner->controlTrajectory((*node)->getVec(), nearNode->getVec()))
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
bool PRMPlanner::queryPath(Node startNode, Node goalNode) {
    if (startNode.empty() || goalNode.empty()) {
        sendMessage("Start or goal node is empty", Message::warning);
        return false;
    }

    shared_ptr<Node> sourceNode = connectNode(startNode);
    shared_ptr<Node> targetNode = connectNode(goalNode);
    if (sourceNode == nullptr || targetNode == nullptr) {
        sendMessage("Start or goal Node could not be connected", Message::warning);
        return false;
    }

    bool pathPlanned = aStar(sourceNode, targetNode);

    if (pathPlanned) {
        this->sendMessage("Path could be planned", Message::info);
        m_nodePath.push_back(shared_ptr<Node>(new Node(goalNode)));
        shared_ptr<Node> temp = targetNode;
        int count = 0;
        while (temp != nullptr) {
            ++count;
            m_nodePath.push_back(temp);
            temp = temp->getParent();
        }
        m_nodePath.push_back(shared_ptr<Node>(new Node(startNode)));
    } else {
        this->sendMessage("Path could NOT be planned", Message::warning);
    }
}

/*!
*  \brief      Try to find nearest Node of the graph to the passed Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[in]  nearest Node
*  \date       2016-08-09
*/
shared_ptr<Node> PRMPlanner::connectNode(Node &node) {
    std::vector<shared_ptr<Node>> nearNodes = this->m_graph->getNearNodes(shared_ptr<Node>(new Node(node)), m_rangeSize * 3);
    float dist = std::numeric_limits<float>::max();
    shared_ptr<Node> nearestNode = nullptr;
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (this->m_planner->controlTrajectory(node, *nearNodes[i]) && node.getDist(nearNodes[i]) < dist) {
            dist = node.getDist(nearNodes[i]);
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
    std::vector<Edge> edges = sourceNode->getChildEdges();
    for (int i = 0; i < edges.size(); ++i) {
        edges[i].getTarget()->setCost(edges[i].getLength());
        edges[i].getTarget()->setParent(sourceNode);
        m_openList.addNode(edges[i].getTarget());
    }
    m_closedList.addNode(sourceNode);

    int count = 0;
    shared_ptr<Node> currentNode;
    while (!m_openList.empty()) {
        currentNode = m_openList.removeMin();

        if (currentNode == targetNode) {
            std::cout << "closed list has: " << count << std::endl;
            return true;
        }

        m_closedList.addNode(currentNode);
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
        if (m_closedList.contains(successor))
            continue;

        float dist = currentNode->getCost() + currentNode->getDist(successor);

        if (m_openList.contains(successor) && dist >= successor->getCost())
            continue;

        successor->setParent(currentNode);
        successor->setCost(dist);
        if (m_openList.contains(successor)) {
            // nothing to do
        } else {
            m_openList.addNode(successor);
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
std::vector<Vec<float>> PRMPlanner::getPath(float trajectoryStepSize, bool smoothing) {
    return getPathFromNodes(m_nodePath, trajectoryStepSize, smoothing);
}
