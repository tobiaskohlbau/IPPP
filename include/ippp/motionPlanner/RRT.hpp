//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/motionPlanner/TreePlanner.hpp>
#include <ippp/motionPlanner/options/RRTOptions.hpp>

namespace ippp {

/*!
* \brief   Super class of the RRT
* \author  Sascha Kaden
*  \date   2017-06-20
*/
template <unsigned int dim>
class RRT : public TreePlanner<dim> {
  public:
    RRT(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options, const std::shared_ptr<Graph<dim>> &graph,
        const std::string &name = "RRT");

    virtual bool expand(size_t nbOfNodes, size_t nbOfThreads = 1);
    virtual bool connectGoalNode(const Vector<dim> goal);

  protected:
    void computeTreeThread(size_t nbOfNodes);
    virtual std::shared_ptr<Node<dim>> computeRRTNode(const Vector<dim> &randVec);
    Vector<dim> computeNodeNew(const Vector<dim> &randNode, const Vector<dim> &nearestNode);

    // variables
    double m_stepSize = 1;
    double m_simplifiedStepSize = 1;
    std::mutex m_mutex;

    using MotionPlanner<dim>::m_environment;
    using MotionPlanner<dim>::m_graph;
    using MotionPlanner<dim>::m_metric;
    using MotionPlanner<dim>::m_options;
    using MotionPlanner<dim>::m_pathPlanned;
    using MotionPlanner<dim>::m_trajectory;
    using MotionPlanner<dim>::m_sampling;
    using MotionPlanner<dim>::m_validityChecker;
    using TreePlanner<dim>::m_initNode;
    using TreePlanner<dim>::m_goalNode;
};

/*!
*  \brief      Standard constructor of the RRT
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  options
*  \param[in]  graph
*  \param[in]  name
*  \date       2017-06-20
*/
template <unsigned int dim>
RRT<dim>::RRT(const std::shared_ptr<Environment> &environment, const RRTOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph, const std::string &name)
    : TreePlanner<dim>(name, environment, options, graph),
      m_stepSize(options.getStepSize()),
      m_simplifiedStepSize(options.getStepSize()) {
    m_metric->simplifyDist(m_simplifiedStepSize);
}

/*!
*  \brief      Compute tree of the RRT
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] return check of the constraints
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRT<dim>::expand(size_t nbOfNodes, size_t nbOfThreads) {
    if (m_initNode == nullptr) {
        Logging::error("Init Node is not connected", this);
        return false;
    }

    size_t countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        computeTreeThread(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&RRT::computeTreeThread, this, countNodes));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }

    return true;
}

/*!
*  \brief      Compute tree of the RRT, threaded function
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \date       2016-05-27
*/
template <unsigned int dim>
void RRT<dim>::computeTreeThread(size_t nbOfNodes) {
    Vector<dim> sample;
    std::shared_ptr<Node<dim>> newNode;
    for (size_t i = 0; i < nbOfNodes; ++i) {
        sample = m_sampling->getSample();
        if (util::empty<dim>(sample))
            continue;

        newNode = computeRRTNode(sample);
        if (newNode == nullptr)
            continue;
        m_graph->addNode(newNode);
    }
}

/*!
*  \brief      Connects goal Node to tree, if connection is possible
*  \author     Sascha Kaden
*  \param[in]  goal Node
*  \param[out] true, if the connection is valid
*  \date       2016-05-27
*/
template <unsigned int dim>
bool RRT<dim>::connectGoalNode(Vector<dim> goal) {
    if (!m_validityChecker->check(goal)) {
        Logging::warning("Goal configuration isn't valid", this);
        return false;
    }

    std::shared_ptr<Node<dim>> goalNode(new Node<dim>(goal));
    auto nearNodes = m_graph->getNearNodes(*goalNode, m_stepSize * 3);

    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    for (auto &node : nearNodes) {
        if (m_validityChecker->check(m_trajectory->calcTrajBin(goal, node->getValues()))) {
            nearestNode = node;
            break;
        }
    }

    if (nearestNode != nullptr) {
        goalNode->setParent(nearestNode, this->m_metric->calcDist(*nearestNode, *goalNode));
        m_goalNode = goalNode;
        m_graph->addNode(goalNode);
        m_pathPlanned = true;
        Logging::info("Goal is connected", this);
        return true;
    }

    Logging::warning("Goal could NOT connected", this);

    return false;
}

/*!
*  \brief      Computation of the new Node by the normal RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Vec
*  \param[out] new Node
*  \date       2016-06-02
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> RRT<dim>::computeRRTNode(const Vector<dim> &randConfig) {
    auto nearestNode = m_graph->getNearestNode(randConfig);

    Vector<dim> newConfig = computeNodeNew(randConfig, nearestNode->getValues());
    auto newNode = std::make_shared<Node<dim>>(newConfig);

    if (!m_validityChecker->check(newNode->getValues()) ||
        !m_validityChecker->check(m_trajectory->calcTrajBin(*newNode, *nearestNode)))
        return nullptr;

    double edgeCost = m_metric->calcDist(*nearestNode, *newNode);
    newNode->setParent(nearestNode, edgeCost);

    m_mutex.lock();
    nearestNode->addChild(newNode, edgeCost);
    m_mutex.unlock();
    return newNode;
}

/*!
*  \brief      Computation of a new Node from the RRT algorithm
*  \author     Sascha Kaden
*  \param[in]  random Node
*  \param[in]  nearest Node
*  \param[out] new Node
*  \date       2016-05-27
*/
template <unsigned int dim>
Vector<dim> RRT<dim>::computeNodeNew(const Vector<dim> &randConfig, const Vector<dim> &nearestConfig) {
    if (m_metric->calcSimpleDist(randConfig, nearestConfig) < m_simplifiedStepSize)
        return randConfig;

    // p = a + k * (b-a)
    // ||u|| = ||b - a||
    // k = stepSize / ||u||
    Vector<dim> u(randConfig - nearestConfig);
    u *= m_stepSize / u.norm();
    u += nearestConfig;
    return u;
}

} /* namespace ippp */

#endif /* RRT_HPP */
