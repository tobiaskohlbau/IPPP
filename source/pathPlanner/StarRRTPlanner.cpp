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
    shared_ptr<Node> nearestNode = this->m_graph->getNearestNode(Node(randVec));
    // set node new fix fixed step size of 10
    Vec<float> newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getVec());
    newNode = shared_ptr<Node>(new Node(newVec));

    std::vector<shared_ptr<Node>> nearNodes;
    chooseParent(newNode, nearestNode, nearNodes);

    if (this->m_collision->controlCollision(newNode->getVec())) {
        newNode = nullptr;
        return;
    } else if (!this->m_planner->controlTrajectory(newNode->getVec(), nearestNode->getVec())) {
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
    nearNodes = this->m_graph->getNearNodes(newNode, this->m_stepSize);

    float nearestNodeCost = nearestNode->getCost();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < nearestNodeCost) {
            if (this->m_planner->controlTrajectory(newNode->getVec(), nearNodes[i]->getVec())) {
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
                if (this->m_planner->controlTrajectory(newNode->getVec(), nearNode->getVec())) {
                    float cost = nearNode->getCost() - nearNode->getDist(nearNode->getParent());
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
bool StarRRTPlanner::connectGoalNode(Node goal) {
    if (this->m_collision->controlCollision(goal.getVec()))
        return false;

    shared_ptr<Node> goalNode(new Node(goal));
    std::vector<shared_ptr<Node>> nearNodes = this->m_graph->getNearNodes(goalNode, this->m_stepSize * 3);

    shared_ptr<Node> nearestNode = nullptr;
    float minCost = std::numeric_limits<float>::max();
    for (int i = 0; i < nearNodes.size(); ++i) {
        if (nearNodes[i]->getCost() < minCost) {
            if (this->m_planner->controlTrajectory(goalNode->getVec(), nearNodes[i]->getVec())) {
                minCost = nearNodes[i]->getCost();
                nearestNode = nearNodes[i];
            }
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode);
        this->m_goalNode = goalNode;
        this->m_goalNode->setCost(m_goalNode->getDist(*nearestNode) + nearestNode->getCost());
        // this->sendMessage("Goal Node is connected", Message::info);
        this->m_pathPlanned = true;
        return true;
    }

    // this->sendMessage("Goal Node is NOT connected", Message::warning);

    return false;
}