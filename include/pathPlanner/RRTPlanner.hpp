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

#ifndef RRTPLANNER_HPP
#define RRTPLANNER_HPP

#include "Planner.hpp"
#include <pathPlanner/options/RRTOptions.hpp>

namespace rmpl {



/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class RRTPlanner : public Planner<dim> {
  public:
    RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions<dim> &options);

    bool computePath(Vector<dim> start, Vector<dim> goal, unsigned int numNodes, unsigned int numThreads);
    bool setInitNode(Vector<dim> start);
    bool computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    virtual bool connectGoalNode(Vector<dim> goal) = 0;

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(float trajectoryStepSize, bool smoothing = true);
    std::shared_ptr<Node<dim>> getInitNode();
    std::shared_ptr<Node<dim>> getGoalNode();

  protected:
    void computeTreeThread(unsigned int nbOfNodes);
    virtual void computeRRTNode(const Vector<dim> &randVec, std::shared_ptr<Node<dim>> &newNode) = 0;
    Vector<dim> computeNodeNew(const Vector<dim> &randNode, const Vector<dim> &nearestNode);

    // variables
    float m_stepSize;
    std::shared_ptr<Node<dim>> m_initNode;
    std::shared_ptr<Node<dim>> m_goalNode;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_planner;
    using Planner<dim>::m_robot;
    using Planner<dim>::m_sampler;
};

/*!
*  \brief      Constructor of the class RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \param[in]  options
*  \date       2016-05-27
*/
template <unsigned int dim>
RRTPlanner<dim>::RRTPlanner(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions<dim> &options)
    : Planner<dim>(name, robot, options) {
    m_stepSize = options.getStepSize();
    m_initNode = nullptr;
    m_goalNode = nullptr;
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
bool RRTPlanner<dim>::computePath(Vector<dim> start, Vector<dim> goal, unsigned int numNodes, unsigned int numThreads) {
    if (!setInitNode(start))
        return false;

    computeTree(numNodes, numThreads);

    return connectGoalNode(goal);
}

/*!
*  \brief      Set init Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if set was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRTPlanner<dim>::setInitNode(Vector<dim> start) {
    if (m_collision->controlVec(start)) {
        Logging::warning("Init Node<dim> could not be connected", this);
        return false;
    }

    std::shared_ptr<Node<dim>> initNode(new Node<dim>(start));
    m_sampler->setMeanOfDistribution(start);
    m_initNode = initNode;
    m_graph->addNode(m_initNode);
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
        Logging::error("Init Node<dim> is not connected", this);
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
        Vector<dim> randVec = m_sampler->getSample();
        std::shared_ptr<Node<dim>> newNode;
        computeRRTNode(randVec, newNode);

        if (newNode == nullptr)
            continue;
        m_graph->addNode(newNode);
    }
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> RRTPlanner<dim>::getPathNodes() {
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    if (!m_pathPlanned)
        return nodes;

    nodes.push_back(m_goalNode);
    for (std::shared_ptr<Node<dim>> temp = m_goalNode->getParentNode(); temp != nullptr; temp = temp->getParentNode())
        nodes.push_back(temp);

    Logging::info("Path has: " + std::to_string(nodes.size()) + " nodes", this);
    return nodes;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[out] configurations of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<Vector<dim>> RRTPlanner<dim>::getPath(float trajectoryStepSize, bool smoothing) {
    std::vector<Vector<dim>> path;
    if (!m_pathPlanned) {
        Logging::warning("Path is not complete", this);
        return path;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = getPathNodes();
    path = this->getPathFromNodes(nodes, trajectoryStepSize, smoothing);

    Logging::info("Path has: " + std::to_string(path.size()) + " points", this);
    return path;
}

/*!
*  \brief      Computation of a new Node<dim> from the RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Node
*  \param[in]  nearest Node
*  \param[out] new Node
*  \date       2016-05-27
*/
template <unsigned int dim>
Vector<dim> RRTPlanner<dim>::computeNodeNew(const Vector<dim> &randNode, const Vector<dim> &nearestNode) {
    if ((randNode - nearestNode).norm() < m_stepSize)
        return randNode;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vector<dim> u = randNode - nearestNode;
    u *= m_stepSize / u.norm();
    u += nearestNode;
    return u;
}

/*!
*  \brief      Return the init Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] init Node
*  \date       2016-06-01
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRTPlanner<dim>::getInitNode() {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
*  \date       2016-06-01
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRTPlanner<dim>::getGoalNode() {
    return m_goalNode;
}

} /* namespace rmpl */

#endif /* RRTPLANNER_HPP */
