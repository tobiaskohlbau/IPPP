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

#include <pathPlanner/StarRRTPlanner.h>

#include <include/core/utility/Logging.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief         Computation of the new Node by the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
void StarRRTPlanner::computeRRTNode(const Vec<float> &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = m_graph->getNearestNode(Node(randVec));
    // set node new fix fixed step size of 10
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    std::vector<shared_ptr<Node>> nearNodes;
    chooseParent(newNode, nearestNode, nearNodes);

    if (m_collision->controlVec(newNode->getVec())) {
        newNode = nullptr;
        return;
    } else if (!m_planner->controlTrajectory(newNode, nearestNode)) {
        newNode = nullptr;
        return;
    }

    newNode->setCost(newNode->getDist(*nearestNode) + nearestNode->getCost());
    newNode->setParent(nearestNode);
    m_mutex.lock();
    nearestNode->addChild(newNode);
    m_mutex.unlock();

    reWire(newNode, nearestNode, nearNodes);
}

/*!
*  \brief         Choose parent algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] nearest Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
void StarRRTPlanner::chooseParent(shared_ptr<Node> &newNode, shared_ptr<Node> &nearestNode,
                                  std::vector<shared_ptr<Node>> &nearNodes) {
    // get near nodes to the new node
    nearNodes = m_graph->getNearNodes(newNode, m_stepSize);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost) {
            if (m_planner->controlTrajectory(newNode, nearNodes[i])) {
                nearestNodeCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
        }
    }
}

/*!
*  \brief         Rewire algorithm from the RRT* algorithm
*  \author        Sascha Kaden
*  \param[in,out] new Node
*  \param[in,out] parent Node
*  \param[in,out] vecotr of nearest nodes
*  \date          2016-06-02
*/
void StarRRTPlanner::reWire(shared_ptr<Node> &newNode, shared_ptr<Node> &parentNode, std::vector<shared_ptr<Node>> &nearNodes) {
    for (auto nearNode : nearNodes) {
        if (nearNode != parentNode && nearNode->getChildEdges().size() != 0) {
            float oldDist = nearNode->getCost();
            float newDist = nearNode->getDist(newNode) + newNode->getCost();
            if (newDist < oldDist) {
                if (m_planner->controlTrajectory(newNode, nearNode)) {
                    float cost = nearNode->getCost() - nearNode->getDist(nearNode->getParentNode());
                    cost += newNode->getDist(nearNode);
                    m_mutex.lock();
                    nearNode->setCost(cost);
                    nearNode->setParent(newNode);
                    m_mutex.unlock();
                }
            }
        }
    }
}

/*!
*  \brief      Connects goal Node to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
bool StarRRTPlanner::connectGoalNode(Vec<float> goal) {
    if (m_collision->controlVec(goal))
        return false;

    shared_ptr<Node> goalNode(new Node(goal));
    std::vector<shared_ptr<Node>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    shared_ptr<Node> nearestNode = nullptr;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost && m_planner->controlTrajectory(goalNode, nearNodes[i])) {
            minCost = nearNodes[i]->getCost();
            nearestNode = nearNodes[i];
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode);
        m_goalNode = goalNode;
        m_goalNode->setCost(m_goalNode->getDist(*nearestNode) + nearestNode->getCost());
        // Logging::info("Goal Node is connected", this);
        m_pathPlanned = true;
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}
