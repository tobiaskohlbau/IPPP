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

#include <pathPlanner/RRTPlanner.h>

#include <thread>

#include <core/Logging.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class RRTPlanner
*  \author     Sascha Kaden
*  \date       2016-05-27
*/
RRTPlanner::RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize,
                       TrajectoryMethod trajectory, SamplingMethod sampling)
    : Planner(name, robot, trajectoryStepSize, trajectory, sampling) {
    m_stepSize = stepSize;
    m_initNode = nullptr;
    m_goalNode = nullptr;
}

/*!
*  \brief      Set init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if set was possible
*  \date       2016-05-27
*/
bool RRTPlanner::setInitNode(Node node) {
    shared_ptr<Node> initNode(new Node(node));
    if (this->m_collision->controlCollision(initNode->getVec())) {
        Logging::warning("Init node could not be connected", this);
        return false;
    }

    m_initNode = initNode;
    this->m_graph->addNode(m_initNode);
    return true;
}

/*!
*  \brief      Compute tree of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] return check of the constraints
*  \date       2016-05-27
*/
bool RRTPlanner::computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads) {
    if (!controlConstraints())
        return false;

    if (nbOfThreads == 1) {
        computeTreeThread(nbOfNodes);
    } else {
        nbOfNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (int i = 0; i < nbOfThreads; ++i) {
            threads.push_back(std::thread(&RRTPlanner::computeTreeThread, this, nbOfNodes));
        }

        for (int i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }

    return true;
}

/*!
*  \brief      Compute tree of the RRTPlanner, threaded function
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \date       2016-05-27
*/
void RRTPlanner::computeTreeThread(unsigned int nbOfNodes) {
    for (int i = 0; i < nbOfNodes; ++i) {
        Vec<float> randVec = this->m_sampler->getSample(m_robot->getDim(), i, nbOfNodes);
        shared_ptr<Node> newNode;
        computeRRTNode(randVec, newNode);

        if (newNode == nullptr)
            continue;
        this->m_graph->addNode(newNode);
    }
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
std::vector<std::shared_ptr<Node>> RRTPlanner::getPathNodes() {
    std::vector<shared_ptr<Node>> nodes;
    if (!this->m_pathPlanned)
        return nodes;

    nodes.push_back(m_goalNode);
    shared_ptr<Node> temp = m_goalNode->getParent();
    while (temp != nullptr) {
        nodes.push_back(temp);
        temp = temp->getParent();
    }
    Logging::info("Path has: " + std::to_string(nodes.size()) + " nodes", this);
    return nodes;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[out] vecs of the path
*  \date       2016-05-31
*/
std::vector<Vec<float>> RRTPlanner::getPath(float trajectoryStepSize, bool smoothing) {
    std::vector<Vec<float>> path;
    if (!this->m_pathPlanned) {
        Logging::warning("Path is not complete", this);
        return path;
    }

    std::vector<std::shared_ptr<Node>> nodes = getPathNodes();
    path = this->getPathFromNodes(nodes, trajectoryStepSize, smoothing);

    Logging::info("Path has: " + std::to_string(path.size()) + " points", this);
    return path;
}

/*!
*  \brief      Computation of a new Node from the RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Node
*  \param[in]  nearest Node
*  \param[out] Node new
*  \date       2016-05-27
*/
Vec<float> RRTPlanner::computeNodeNew(const Vec<float> &randNode, const Vec<float> &nearestNode) {
    if (randNode.getDist(nearestNode) < this->m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vec<float> u = randNode - nearestNode;
    u *= this->m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

/*!
*  \brief      Return the init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] init Node
* \date        2016-06-01
*/
std::shared_ptr<Node> RRTPlanner::getInitNode() {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
* \date        2016-06-01
*/
std::shared_ptr<Node> RRTPlanner::getGoalNode() {
    return m_goalNode;
}

/*!
*  \brief      Return sample point of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] sample Vec
* \date        2016-07-22
*/
Vec<float> RRTPlanner::getSamplePoint() {
    return this->m_sampler->getSample(m_robot->getDim(), 0, 100);
}

/*!
*  \brief      Control all constraints of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] check flag
*  \date       2016-05-27
*/
bool RRTPlanner::controlConstraints() {
    if (m_initNode == nullptr) {
        Logging::warning("Init node is not connected", this);
        return false;
    } else {
        return true;
    }
}
