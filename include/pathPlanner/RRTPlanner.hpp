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

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "Planner.hpp"
#include <pathPlanner/options/RRTOptions.h>

namespace rmpl {

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class RRTPlanner : public Planner<dim> {
  public:
    RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const RRTOptions &options);

    bool computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads);
    bool setInitNode(Eigen::VectorXf start);
    bool computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    virtual bool connectGoalNode(Eigen::VectorXf goal) = 0;

    std::vector<std::shared_ptr<Node>> getPathNodes();
    std::vector<Eigen::VectorXf> getPath(float trajectoryStepSize, bool smoothing = true);
    std::shared_ptr<Node> getInitNode();
    std::shared_ptr<Node> getGoalNode();

  protected:
    void computeTreeThread(unsigned int nbOfNodes);
    virtual void computeRRTNode(const Eigen::VectorXf &randVec, std::shared_ptr<Node> &newNode) = 0;
    Eigen::VectorXf computeNodeNew(const Eigen::VectorXf &randNode, const Eigen::VectorXf &nearestNode);

    // variables
    float m_stepSize;
    std::shared_ptr<Node> m_initNode;
    std::shared_ptr<Node> m_goalNode;
};

/*!
*  \brief      Constructor of the class RRTPlanner
*  \author     Sascha Kaden
*  \date       2016-05-27
*/
template <unsigned int dim>
RRTPlanner<dim>::RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase> &robot, const RRTOptions &options)
    : Planner<dim>(name, robot, options) {
    m_stepSize = options.getStepSize();
    m_initNode = nullptr;
    m_goalNode = nullptr;
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
bool RRTPlanner<dim>::computePath(Eigen::VectorXf start, Eigen::VectorXf goal, unsigned int numNodes, unsigned int numThreads) {
    if (!setInitNode(start))
        return false;

    computeTree(numNodes, numThreads);

    return connectGoalNode(goal);
}

/*!
*  \brief      Set init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if set was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRTPlanner<dim>::setInitNode(Eigen::VectorXf start) {
    if (this->m_collision->controlVec(start)) {
        Logging::warning("Init node could not be connected", this);
        return false;
    }

    std::shared_ptr<Node> initNode(new Node(start));
    this->m_sampler->setMeanOfDistribution(start);
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
template <unsigned int dim>
bool RRTPlanner<dim>::computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads) {
    if (m_initNode == nullptr) {
        Logging::error("Init node is not connected", this);
        return false;
    }

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
template <unsigned int dim>
void RRTPlanner<dim>::computeTreeThread(unsigned int nbOfNodes) {
    for (int i = 0; i < nbOfNodes; ++i) {
        Eigen::VectorXf randVec = this->m_sampler->getSample();
        std::shared_ptr<Node> newNode;
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
template <unsigned int dim>
std::vector<std::shared_ptr<Node>> RRTPlanner<dim>::getPathNodes() {
    std::vector<std::shared_ptr<Node>> nodes;
    if (!this->m_pathPlanned)
        return nodes;

    nodes.push_back(m_goalNode);
    std::shared_ptr<Node> temp = m_goalNode->getParentNode();
    while (temp != nullptr) {
        nodes.push_back(temp);
        temp = temp->getParentNode();
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
template <unsigned int dim>
std::vector<Eigen::VectorXf> RRTPlanner<dim>::getPath(float trajectoryStepSize, bool smoothing) {
    std::vector<Eigen::VectorXf> path;
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
template <unsigned int dim>
Eigen::VectorXf RRTPlanner<dim>::computeNodeNew(const Eigen::VectorXf &randNode, const Eigen::VectorXf &nearestNode) {
    if ((randNode - nearestNode).norm() < m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Eigen::VectorXf u = randNode - nearestNode;
    u *= m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

/*!
*  \brief      Return the init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] init Node
* \date        2016-06-01
*/
template <unsigned int dim>
std::shared_ptr<Node> RRTPlanner<dim>::getInitNode() {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
* \date        2016-06-01
*/
template <unsigned int dim>
std::shared_ptr<Node> RRTPlanner<dim>::getGoalNode() {
    return m_goalNode;
}

} /* namespace rmpl */

#endif /* RRTPLANNER_H_ */
