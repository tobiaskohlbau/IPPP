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

#include <pathPlanner/NormalRRTPlanner.h>

#include <core/utility/Logging.h>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief         Computation of the new Node by the normal RRT algorithm
*  \author        Sascha Kaden
*  \param[in]     random Vec
*  \param[in,out] new Node
*  \date          2016-06-02
*/
void NormalRRTPlanner::computeRRTNode(const Eigen::VectorXf &randVec, shared_ptr<Node> &newNode) {
    // get nearest neighbor
    shared_ptr<Node> nearestNode = m_graph->getNearestNode(Node(randVec));

    // compute node new with fixed step size
    Eigen::VectorXf newVec = RRTPlanner::computeNodeNew(randVec, nearestNode->getValues());
    newNode = shared_ptr<Node>(new Node(newVec));

    if (m_collision->controlVec(newNode->getValues())) {
        newNode = nullptr;
        return;
    } else if (!m_planner->controlTrajectory(newNode->getValues(), nearestNode->getValues())) {
        newNode = nullptr;
        return;
    }

    newNode->setParent(nearestNode);

    m_mutex.lock();
    nearestNode->addChild(newNode);
    m_mutex.unlock();
}

/*!
*  \brief      Connects goal Node to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
bool NormalRRTPlanner::connectGoalNode(Eigen::VectorXf goal) {
    if (m_collision->controlVec(goal))
        return false;

    shared_ptr<Node> goalNode(new Node(goal));
    std::vector<shared_ptr<Node>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    shared_ptr<Node> nearestNode = nullptr;
    for (auto node : nearNodes) {
        if (m_planner->controlTrajectory(goal, node->getValues())) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode);
        m_goalNode = goalNode;
        m_graph->addNode(goalNode);
        // Logging::info("Goal Node is connected", this);
        m_pathPlanned = true;
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}

} /* namespace rmpl */