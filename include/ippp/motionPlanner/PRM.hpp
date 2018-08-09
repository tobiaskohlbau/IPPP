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

#ifndef PRM_HPP
#define PRM_HPP

#include <ippp/motionPlanner/MotionPlanner.hpp>
#include <ippp/motionPlanner/options/PRMOptions.hpp>

namespace ippp {

/*!
* \brief   Class PRM
* \author  Sascha Kaden
* \date    2016-08-09
*/
template <unsigned int dim>
class PRM : public MotionPlanner<dim> {
  public:
    PRM(const std::shared_ptr<Environment> &environment, const PRMOptions<dim> &options,
        const std::shared_ptr<Graph<dim>> &graph);

    bool computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads = 1);
    bool computePath(const Vector<dim> start, const std::vector<Vector<dim>> goals, size_t numNodes, size_t numThreads = 1);
    bool computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, const std::pair<Vector6, Vector6> &C,
                           size_t numNodes, size_t numThreads);
    bool computePathToPose(const Vector<dim> startConfig, const std::vector<Vector6> pathPoses,
                           const std::pair<Vector6, Vector6> &C, size_t numNodes, size_t numThreads = 1);
    bool expand(size_t numNodes, size_t numThreads);

    void startSamplingPhase(size_t nbOfNodes, size_t nbOfThreads = 1);
    void startPlannerPhase(size_t nbOfThreads = 1);

    bool queryPath(const Vector<dim> start, const Vector<dim> goal);

    std::vector<std::shared_ptr<Node<dim>>> getPathNodes();
    std::vector<Vector<dim>> getPath(double posRes = 1, double oriRes = 0.1);

  protected:
    void samplingPhase(size_t nbOfNodes);
    void plannerPhase(size_t startNodeIndex, size_t endNodeIndex);
    std::shared_ptr<Node<dim>> connectNode(const Vector<dim> &config);

    double m_rangeSize;
    std::vector<std::shared_ptr<Node<dim>>> m_nodePath;

    using MotionPlanner<dim>::m_validityChecker;
    using MotionPlanner<dim>::m_environment;
    using MotionPlanner<dim>::m_evaluator;
    using MotionPlanner<dim>::m_graph;
    using MotionPlanner<dim>::m_metric;
    using MotionPlanner<dim>::m_options;
    using MotionPlanner<dim>::m_pathPlanned;
    using MotionPlanner<dim>::m_plannerCollector;
    using MotionPlanner<dim>::m_trajectory;
    using MotionPlanner<dim>::m_sampling;
    using MotionPlanner<dim>::updateStats;
    using MotionPlanner<dim>::initParams;
};

/*!
*  \brief      Standard constructor of the class PRM
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  PRMOptions
*  \param[in]  Graph
*  \date       2016-08-09
*/
template <unsigned int dim>
PRM<dim>::PRM(const std::shared_ptr<Environment> &environment, const PRMOptions<dim> &options,
              const std::shared_ptr<Graph<dim>> &graph)
    : MotionPlanner<dim>("PRM", environment, options, graph) {
    m_rangeSize = options.getRangeSize();
}

/*!
*  \brief      Compute path from start Node to goal Node with passed number of samples and threads
*  \author     Sascha Kaden
*  \param[in]  start configuration
*  \param[in]  goal configuration
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] true, if path was found
*  \date       2016-05-27
*/
template <unsigned int dim>
bool PRM<dim>::computePath(const Vector<dim> start, const Vector<dim> goal, size_t numNodes, size_t numThreads) {
    m_plannerCollector->startPlannerTimer();
    std::vector<Vector<dim>> query = {start, goal};
    if (!m_validityChecker->check(query)) {
        Logging::error("Configurations are not valid!", this);
        return false;
    }
    m_evaluator->setConfigs(query);
    initParams(start, goal);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        expand(numNodes, numThreads);
    }

    m_plannerCollector->stopPlannerTimer();
    updateStats();
    return queryPath(start, goal);
}

template <unsigned int dim>
bool PRM<dim>::computePath(const Vector<dim> start, const std::vector<Vector<dim>> goals, size_t numNodes, size_t numThreads) {
    m_plannerCollector->startPlannerTimer();

    std::vector<Vector<dim>> query = goals;
    query.insert(query.begin(), start);
    if (!m_validityChecker->check(query)) {
        Logging::error("Configurations are not valid!", this);
        return false;
    }
    m_evaluator->setConfigs(query);
    initParams(start, goals[0]);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        expand(numNodes, numThreads);
    }
    updateStats();

    std::vector<std::shared_ptr<Node<dim>>> nodePath;
    for (auto config = query.begin(); config < query.end() - 1; ++config) {
        if (!queryPath(*config, *(config + 1)))
            return false;
        nodePath.insert(nodePath.end(), m_nodePath.begin(), m_nodePath.end());
    }
    m_nodePath = nodePath;

    m_plannerCollector->stopPlannerTimer();
    return true;
}

template <unsigned int dim>
bool PRM<dim>::computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, const std::pair<Vector6, Vector6> &C,
                                 size_t numNodes, size_t numThreads) {
    m_plannerCollector->startPlannerTimer();
    // this->setSamplingParams(start, goal);
    if (!m_validityChecker->check(startConfig)) {
        Logging::error("start configurations is not valid!", this);
        return false;
    }

    m_evaluator->setConfigs(std::vector<Vector<dim>>({startConfig}));
    m_evaluator->setPoses(std::vector<Vector6>({goalPose}));

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        expand(numNodes, numThreads);
    }

    // set goal node
    Vector<dim> goalConfig = util::NaNVector<dim>();
    auto robot = m_environment->getRobot();
    for (auto &node : m_graph->getNodes()) {
        if (util::checkConfigToPose<dim>(node->getValues(), goalPose, *robot, C)) {
            goalConfig = node->getValues();
            break;
        }
    }
    if (util::empty<dim>(goalConfig)) {
        Logging::warning("No goal configuration to passed goal pose found!", this);
        return false;
    }

    m_plannerCollector->stopPlannerTimer();
    updateStats();
    return queryPath(startConfig, goalConfig);
}

template <unsigned int dim>
bool PRM<dim>::computePathToPose(const Vector<dim> startConfig, const std::vector<Vector6> pathPoses,
                                 const std::pair<Vector6, Vector6> &C, size_t numNodes, size_t numThreads) {
    m_plannerCollector->startPlannerTimer();
    if (!m_validityChecker->check(startConfig)) {
        Logging::error("start configurations is not valid!", this);
        return false;
    }

    m_evaluator->setConfigs(std::vector<Vector<dim>>({startConfig}));
    m_evaluator->setPoses(pathPoses);

    size_t loopCount = 1;
    while (!m_evaluator->evaluate()) {
        Logging::debug("Iteration: " + std::to_string(loopCount++), this);
        expand(numNodes, numThreads);
    }

    updateStats();
    // set the goal configurations
    std::vector<Vector<dim>> configs(pathPoses.size(), util::NaNVector<dim>());    // = { startConfig };
    auto robot = m_environment->getRobot();
    for (auto &node : m_graph->getNodes()) {
        for (size_t i = 0; i < pathPoses.size(); ++i) {
            if (util::checkConfigToPose<dim>(node->getValues(), pathPoses[i], *robot, C)) {
                configs[i] = node->getValues();
                break;
            }
        }
    }
    for (auto &config : configs) {
        if (util::empty<dim>(config)) {
            Logging::warning("No goal configuration to passed goal pose found!", this);
            return false;
        }
    }

    configs.insert(configs.begin(), startConfig);
    std::vector<std::shared_ptr<Node<dim>>> nodePath;
    for (auto config = configs.begin(); config < configs.end() - 1; ++config) {
        if (!queryPath(*config, *(config + 1)))
            return false;
        nodePath.insert(nodePath.end(), m_nodePath.begin(), m_nodePath.end());
    }
    m_nodePath = nodePath;

    m_plannerCollector->stopPlannerTimer();
    return true;
}

/*!
*  \brief      Expands graph
*  \author     Sascha Kaden
*  \param[in]  number of samples
*  \param[in]  number of threads
*  \param[out] error flag
*  \date       2017-03-01
*/
template <unsigned int dim>
bool PRM<dim>::expand(size_t numNodes, size_t numThreads) {
    startSamplingPhase(numNodes, numThreads);
    m_graph->sortTree();
    startPlannerPhase(numThreads);
    return true;
}

/*!
*  \brief      Sampling phase of the PRM
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startSamplingPhase(size_t nbOfNodes, size_t nbOfThreads) {
    size_t countNodes = nbOfNodes;
    if (nbOfThreads == 1) {
        samplingPhase(nbOfNodes);
    } else {
        countNodes /= nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&PRM::samplingPhase, this, countNodes));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Sampling thread function
*  \author     Sascha Kaden
*  \param[in]  number of Nodes to be sampled
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::samplingPhase(size_t nbOfNodes) {
    Vector<dim> sample;
    for (size_t i = 0; i < nbOfNodes; ++i) {
        sample = m_sampling->getSample();
        if (util::empty<dim>(sample))
            continue;
        m_graph->addNode(std::shared_ptr<Node<dim>>(new Node<dim>(sample)));
    }
}

/*!
*  \brief      Local planning phase of the PRM.
*  \details    Add the nearest neighbors of a Node as childes.
*  \author     Sascha Kaden
*  \param[in]  number of threads
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::startPlannerPhase(size_t nbOfThreads) {
    size_t nodeCount = m_graph->numNodes();
    if (nbOfThreads == 1) {
        plannerPhase(0, nodeCount);
    } else {
        size_t threadAmount = nodeCount / nbOfThreads;
        std::vector<std::thread> threads;

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads.push_back(std::thread(&PRM::plannerPhase, this, i * threadAmount, (i + 1) * threadAmount));

        for (size_t i = 0; i < nbOfThreads; ++i)
            threads[i].join();
    }
}

/*!
*  \brief      Local planning thread function
*  \details    Searches the nearest neighbors between the given indexes and adds them as childes
*  \author     Sascha Kaden
*  \param[in]  start index
*  \param[in]  end index
*  \date       2016-08-09
*/
template <unsigned int dim>
void PRM<dim>::plannerPhase(size_t startNodeIndex, size_t endNodeIndex) {
    if (startNodeIndex > endNodeIndex) {
        Logging::error("Start index is larger than end index", this);
        return;
    }

    std::vector<std::shared_ptr<Node<dim>>> nodes = m_graph->getNodes();
    if (endNodeIndex > nodes.size()) {
        Logging::error("End index is larger than Node size", this);
        return;
    }

    for (auto node = nodes.begin() + startNodeIndex; node != nodes.begin() + endNodeIndex; ++node) {
        auto nearNodes = m_graph->getNearNodes(**node, m_rangeSize);
        for (auto &nearNode : nearNodes) {
            if ((*node)->isChild(nearNode) || (*node)->isInvalidChild(nearNode))
                continue;

            if (m_validityChecker->check(m_trajectory->calcTrajBin(**node, *nearNode)))
                (*node)->addChild(nearNode, m_metric->calcDist(*nearNode, **node));
            else
                (*node)->addInvalidChild(nearNode);
        }
    }
}

/*!
*  \brief      Searches a between start and goal Node
*  \details    Uses internal the A* algorithm to find the best path. It saves the path Nodes internal.
*  \author     Sascha Kaden
*  \param[in]  start Node
*  \param[in]  goal Node
*  \param[out] result of query
*  \date       2016-08-09
*/
template <unsigned int dim>
bool PRM<dim>::queryPath(const Vector<dim> start, const Vector<dim> goal) {
    std::shared_ptr<Node<dim>> sourceNode = connectNode(start);
    std::shared_ptr<Node<dim>> goalNode = connectNode(goal);
    if (sourceNode == nullptr || goalNode == nullptr) {
        Logging::info("Start or goal Node could not be connected", this);
        return false;
    }

    m_graph->clearQueryParents();
    m_nodePath.clear();
    if (util::aStar<dim>(sourceNode, goalNode, *m_metric)) {
        Logging::info("Path could be planned", this);

        auto tmpNode = goalNode;
        while (tmpNode != nullptr) {
            m_nodePath.push_back(tmpNode);
            tmpNode = tmpNode->getQueryParentNode();
        }

        std::reverse(m_nodePath.begin(), m_nodePath.end());
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
*  \date       2017-11-16
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> PRM<dim>::connectNode(const Vector<dim> &config) {
    // check that the graph contains the node and if yes return it
    std::shared_ptr<Node<dim>> nearestNode = m_graph->getNode(config);
    if (nearestNode)
        return nearestNode;

    // if not, create a new node and connect near nodes
    auto newNode = std::make_shared<Node<dim>>(config);
    auto nearNodes = m_graph->getNearNodes(config, m_rangeSize);
    for (auto &nearNode : nearNodes) {
        if (m_validityChecker->check(m_trajectory->calcTrajBin(*newNode, *nearNode))) {
            newNode->addChild(nearNode, m_metric->calcDist(*newNode, *nearNode));
            nearNode->addChild(newNode, m_metric->calcDist(*nearNode, *newNode));
        } else {
            newNode->addInvalidChild(nearNode);
        }
    }
    m_graph->addNode(newNode);
    return newNode;
}

/*!
*  \brief      Return all nodes of the final path
*  \author     Sascha Kaden
*  \param[out] nodes of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> PRM<dim>::getPathNodes() {
    return m_nodePath;
}

/*!
*  \brief      Return all points of the final path
*  \author     Sascha Kaden
*  \param[in]  trajectory step size
*  \param[out] configurations of the path
*  \date       2016-05-31
*/
template <unsigned int dim>
std::vector<Vector<dim>> PRM<dim>::getPath(double posRes, double oriRes) {
    return this->getPathFromNodes(m_nodePath, posRes, oriRes);
}

} /* namespace ippp */

#endif    // PRM_HPP
