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

#ifndef MOTIONPLANNER_HPP
#define MOTIONPLANNER_HPP

#include <thread>

#include <Eigen/Core>

#include <ippp/Identifier.h>
#include <ippp/dataObj/Graph.hpp>
#include <ippp/motionPlanner/options/MPOptions.hpp>
#include <ippp/statistic/Stats.h>
#include <ippp/statistic/StatsPathCollector.h>
#include <ippp/statistic/StatsPlannerCollector.h>
#include <ippp/types.h>
#include <ippp/util/UtilEnvironment.hpp>
#include <ippp/util/UtilPlanner.hpp>

namespace ippp {

/*!
* \brief   Super class of all path planners
* \author  Sascha Kaden
* \date    2016-05-27
*/
template <unsigned int dim>
class MotionPlanner : public Identifier {
  public:
    ~MotionPlanner();

  protected:
    MotionPlanner(const std::string &name, const std::shared_ptr<Environment> &environment, const MPOptions<dim> &options,
                  const std::shared_ptr<Graph<dim>> &graph);

  public:
    virtual bool computePath(const Vector<dim> startConfig, const Vector<dim> goalConfig, size_t numNodes,
                             size_t numThreads = 1) = 0;
    virtual bool computePath(const Vector<dim> startConfig, const std::vector<Vector<dim>> pathConfigs, size_t numNodes,
                             size_t numThreads = 1) = 0;
    virtual bool computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, const std::pair<Vector6, Vector6> &C,
                                   size_t numNodes, size_t numThreads = 1) = 0;
    virtual bool computePathToPose(const Vector<dim> startConfig, const std::vector<Vector6> pathPoses,
                                   const std::pair<Vector6, Vector6> &C, size_t numNodes, size_t numThreads = 1) = 0;
    virtual bool expand(size_t numNode, size_t numThreads = 1) = 0;
    virtual bool optimize(size_t numIterations, size_t numNode, size_t numThreads = 1);

    virtual std::vector<Vector<dim>> getPath(double posRes = 1, double oriRes = 0.1) = 0;
    virtual std::vector<std::shared_ptr<Node<dim>>> getPathNodes() = 0;
    std::vector<Vector<dim>> getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes, double posRes, double oriRes);

    std::shared_ptr<Graph<dim>> getGraph();
    std::vector<std::shared_ptr<Node<dim>>> getGraphNodes();
    void updateStats() const;

  protected:
    virtual void initParams(const Vector<dim> &start, const Vector<dim> &goal);

    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<Evaluator<dim>> m_evaluator = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier = nullptr;
    std::shared_ptr<StatsPathCollector> m_pathCollector = nullptr;
    std::shared_ptr<StatsPlannerCollector> m_plannerCollector = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;
    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;

    const MPOptions<dim> m_options;
    bool m_pathPlanned = false; /*!< true flag, if a plan could be computed */
};

/*!
*  \brief      Standard destructor of the Planner
*  \author     Sascha Kaden
*  \date       2016-12-23
*/
template <unsigned int dim>
MotionPlanner<dim>::~MotionPlanner() {
}

/*!
*  \brief      Constructor of the class MotionPlanner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \param[in]  planner options
*  \date       2016-05-27
*/
template <unsigned int dim>
MotionPlanner<dim>::MotionPlanner(const std::string &name, const std::shared_ptr<Environment> &environment,
                                  const MPOptions<dim> &options, const std::shared_ptr<Graph<dim>> &graph)
    : Identifier(name),
      m_pathCollector(std::make_shared<StatsPathCollector>("PathStats")),
      m_plannerCollector(std::make_shared<StatsPlannerCollector>("PlannerStats")),
      m_validityChecker(options.getValidityChecker()),
      m_environment(environment),
      m_evaluator(options.getEvaluator()),
      m_graph(graph),
      m_metric(options.getDistanceMetric()),
      m_options(options),
      m_pathModifier(options.getPathModifier()),
      m_trajectory(options.getTrajectoryPlanner()),
      m_sampling(options.getSampling()) {
    Logging::debug("Initialize", this);
    Stats::addCollector(m_pathCollector);
    Stats::addCollector(m_plannerCollector);

    // check dimensions of the robot to the dimension of the planner
    if (!util::checkDimensions<dim>(*environment))
        Logging::error("Robot dimensions are unequal to planner dimension", this);
    assert(util::checkDimensions<dim>(*environment));

    // check the existance of all modules
    if (m_metric == nullptr || m_environment == nullptr || m_evaluator == nullptr || m_graph == nullptr ||
        m_pathModifier == nullptr || m_pathCollector == nullptr || m_plannerCollector == nullptr || m_sampling == nullptr ||
        m_trajectory == nullptr || m_validityChecker == nullptr)
        Logging::error("Not all modules instanced!", this);
}

template <unsigned int dim>
bool MotionPlanner<dim>::optimize(size_t numIterations, size_t numNodes, size_t numThreads) {
    Logging::warning("No optimization implemented!", this);
    return true;
}

/*!
*  \brief      Return the graph of the path planner
*  \author     Sascha Kaden
*  \param[out] Graph
*  \date       2016-09-27
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> MotionPlanner<dim>::getGraph() {
    return m_graph;
}

/*!
*  \brief      Return all nodes from the Graph
*  \author     Sascha Kaden
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> MotionPlanner<dim>::getGraphNodes() {
    return m_graph->getNodes();
}

/*!
*  \brief      Return path points from passed path nodes, using optional smoothing
*  \author     Sascha Kaden
*  \param[in]  path nodes
*  \param[in]  trajectory step size
*  \param[out] path configurations
*  \date       2016-05-27
*/
template <unsigned int dim>
std::vector<Vector<dim>> MotionPlanner<dim>::getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes, double posRes,
                                                              double oriRes) {
    m_pathCollector->startModificationTimer();
    std::vector<std::shared_ptr<Node<dim>>> smoothedNodes = m_pathModifier->smoothPath(nodes);
    m_pathCollector->stopModificationTimer();

    m_pathCollector->setNodeCounts(nodes.size(), smoothedNodes.size());
    m_pathCollector->setLengths(util::calcPathLength<dim>(nodes, *m_metric), util::calcPathLength<dim>(smoothedNodes, *m_metric));

    Logging::info("Path has after smoothing: " + std::to_string(smoothedNodes.size()) + " nodes", this);

    // save the planner resolution and set passed resolutions
    auto plannerRes = m_trajectory->getResolutions();
    m_trajectory->setResolutions(posRes, oriRes);

    auto smoothedPath = util::calcConfigPath<dim>(smoothedNodes, *m_trajectory);
    m_pathCollector->setConfigCounts(1, smoothedPath.size());

    // set the resolution again to the planner resolution
    m_trajectory->setResolutions(plannerRes);

    return smoothedPath;
}

/*!
*  \brief      Set the Sampling and Sampler params to the used modules
*  \author     Sascha Kaden
*  \param[in]  origin
*  \param[in]  optimal path cost
*  \date       2018-03-04
*/
template <unsigned int dim>
void MotionPlanner<dim>::initParams(const Vector<dim> &start, const Vector<dim> &goal) {
    m_pathPlanned = false;

    if (util::empty<dim>(start)) {
        Logging::error("Empty start configuration at parameter initialization!", this);
        return;
    }
    auto sampler = m_sampling->getSampler();
    sampler->setOrigin(start);

    if (!util::empty<dim>(goal))
        sampler->setOptimalPathCost(m_metric->calcDist(start, goal));
}

/*!
*  \brief      Update all statistics of the planner and his modules.
*  \author     Sascha Kaden
*  \date       2018-03-29
*/
template <unsigned int dim>
void MotionPlanner<dim>::updateStats() const {
    m_graph->updateStats();
}

} /* namespace ippp */

#endif /* MOTIONPLANNER_HPP */