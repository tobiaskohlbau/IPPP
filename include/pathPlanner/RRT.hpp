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

#ifndef RRT_HPP
#define RRT_HPP

#include <mutex>

#include <pathPlanner/Planner.hpp>
#include <pathPlanner/options/RRTOptions.hpp>

namespace rmpl {

/*!
* \brief   Super class of the RRTPlanner
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class RRT : public Planner<dim> {
  public:
    RRT(const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions<dim> &options, const std::string &name = "RRT");

    bool computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes, const unsigned int numThreads);
    bool expand(const unsigned int numNodes, const unsigned int numThreads);
    bool setInitNode(const Vector<dim> start);
    bool computeTree(unsigned int nbOfNodes, unsigned int nbOfThreads = 1);
    virtual bool connectGoalNode(const Vector<dim> goal);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(const float trajectoryStepSize, const bool smoothing = true);
    std::shared_ptr<Node<dim>> getInitNode() const;
    std::shared_ptr<Node<dim>> getGoalNode() const;

  protected:
    void computeTreeThread(unsigned int nbOfNodes);
    virtual std::shared_ptr<Node<dim>> computeRRTNode(const Vector<dim> &randVec);
    Vector<dim> computeNodeNew(const Vector<dim> &randNode, const Vector<dim> &nearestNode);

    // variables
    float m_stepSize = 1;
    std::shared_ptr<Node<dim>> m_initNode = nullptr;
    std::shared_ptr<Node<dim>> m_goalNode = nullptr;

    std::mutex m_mutex;

    using Planner<dim>::m_collision;
    using Planner<dim>::m_graph;
    using Planner<dim>::m_options;
    using Planner<dim>::m_pathPlanned;
    using Planner<dim>::m_trajectory;
    using Planner<dim>::m_robot;
    using Planner<dim>::m_sampling;
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
RRT<dim>::RRT(const std::shared_ptr<RobotBase<dim>> &robot, const RRTOptions<dim> &options, const std::string &name)
    : Planner<dim>("RRT", robot, options) {
    m_stepSize = options.getStepSize();
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
bool RRT<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes,
                           const unsigned int numThreads) {
    if (!setInitNode(start)) {
        return false;
    }
    if (m_collision->controlVec(goal)) {
        Logging::error("Goal Node in collision", this);
        return false;
    }
    computeTree(numNodes, numThreads);

    return connectGoalNode(goal);
}

/*!
*  \brief      Expands tree, if init Node is set.
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if init node is set
*  \date       2017-03-01
*/
template <unsigned int dim>
bool RRT<dim>::expand(const unsigned int numNodes, const unsigned int numThreads) {
    return computeTree(numNodes, numThreads);
}

/*!
*  \brief      Set init Node of the RRTPlanner
*  \author     Sascha Kaden
*  \param[in]  initial Node
*  \param[out] true, if set was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRT<dim>::setInitNode(const Vector<dim> start) {
    if (m_initNode) {
        if (start == m_initNode->getValues()) {
            Logging::info("Equal start node, tree will be expanded", this);
            return true;
        } else {
            Logging::info("New start node, new tree will be created", this);
            m_graph = std::shared_ptr<Graph<dim>>(new Graph<dim>(m_options.getSortCountGraph()));
        }
    }

    if (m_collision->controlVec(start)) {
        Logging::warning("Init Node could not be connected", this);
        return false;
    }

    m_sampling->setOrigin(start);
    m_initNode = std::shared_ptr<Node<dim>>(new Node<dim>(start));
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
bool RRT<dim>::computeTree(const unsigned int nbOfNodes, const unsigned int nbOfThreads) {
    if (m_initNode == nullptr) {
        Logging::error("Init Node is not connected", this);
        return false;
    }

    unsigned int countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        computeTreeThread(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads.push_back(std::thread(&RRT::computeTreeThread, this, countNodes));
        }

        for (unsigned int i = 0; i < nbOfThreads; ++i) {
            threads[i].join();
        }
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
void RRT<dim>::computeTreeThread(const unsigned int nbOfNodes) {
    for (unsigned int i = 0; i < nbOfNodes; ++i) {
        Vector<dim> randVec = m_sampling->getSample();
        std::shared_ptr<Node<dim>> newNode = computeRRTNode(randVec);

        if (newNode == nullptr) {
            continue;
        }
        m_graph->addNode(newNode);
    }
}

/*!
*  \brief      Connects goal Node<dim> to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection was possible
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRT<dim>::connectGoalNode(Vector<dim> goal) {
    if (m_collision->controlVec(goal)) {
        Logging::warning("Goal Node in collision", this);
        return false;
    }

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = m_graph->getNearNodes(goalNode, m_stepSize * 3);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (auto node : nearNodes) {
        if (m_trajectory->controlTrajectory(goal, node->getValues())) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode, this->m_heuristic->calcEdgeCost(nearestNode, goalNode));
        m_goalNode = goalNode;
        m_graph->addNode(goalNode);
        // Logging::info("Goal Node<dim> is connected", this);
        m_pathPlanned = true;
        Logging::info("Goal could connected", this);
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}

/*!
*  \brief      Computation of the new Node<dim> by the normal RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Vec
*  \param[out] new Node
*  \date       2016-06-02
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRT<dim>::computeRRTNode(const Vector<dim> &randVec) {
    // get nearest neighbor
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNearestNode(randVec);

    // compute Node<dim> new with fixed step size
    Vector<dim> newVec = this->computeNodeNew(randVec, nearestNode->getValues());
    std::shared_ptr<Node<dim>> newNode = std::shared_ptr<Node<dim>>(new Node<dim>(newVec));

    if (m_collision->controlVec(newNode->getValues())) {
        return nullptr;
    } else if (!m_trajectory->controlTrajectory(newNode, nearestNode)) {
        return nullptr;
    }

    float edgeCost = this->m_heuristic->calcEdgeCost(nearestNode, newNode);
    newNode->setParent(nearestNode, edgeCost);

    m_mutex.lock();
    nearestNode->addChild(newNode, edgeCost);
    m_mutex.unlock();
    return newNode;
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> RRT<dim>::getPathNodes() {
    std::vector<std::shared_ptr<Node<dim>>> nodes;
    if (!m_pathPlanned) {
        return nodes;
    }

    nodes.push_back(m_goalNode);
    for (std::shared_ptr<Node<dim>> temp = m_goalNode->getParentNode(); temp != nullptr; temp = temp->getParentNode()) {
        nodes.push_back(temp);
    }

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
std::vector<Vector<dim>> RRT<dim>::getPath(const float trajectoryStepSize, const bool smoothing) {
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
Vector<dim> RRT<dim>::computeNodeNew(const Vector<dim> &randNode, const Vector<dim> &nearestNode) {
    if ((randNode - nearestNode).norm() < m_stepSize) {
        return randNode;
    }

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
std::shared_ptr<Node<dim>> RRT<dim>::getInitNode() const {
    return m_initNode;
}

/*!
*  \brief      Return the goal Node<dim> of the RRTPlanner
*  \author     Sascha Kaden
*  \param[out] goal Node
*  \date       2016-06-01
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRT<dim>::getGoalNode() const {
    return m_goalNode;
}

} /* namespace rmpl */

#endif /* RRT_HPP */
