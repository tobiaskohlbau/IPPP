//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <thread>

#include <Eigen/Core>

#include <ippp/Identifier.h>
#include <ippp/dataObj/Graph.hpp>
#include <ippp/planner/options/PlannerOptions.hpp>
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
class Planner : public Identifier {
  public:
    ~Planner();

  protected:
    Planner(const std::string &name, const std::shared_ptr<Environment> &environment, const PlannerOptions<dim> &options,
            const std::shared_ptr<Graph<dim>> &graph);

  public:
    virtual bool computePath(const Vector<dim> startConfig, const Vector<dim> goalConfig, size_t numNodes, size_t numThreads) = 0;
    // virtual bool computePath(const Vector<dim> startConfig, const std::vector<Vector<dim>> pathConfigs, size_t numNodes, size_t
    // numThreads) = 0;
    // virtual bool computePathToPose(const Vector<dim> startConfig, const Vector6 goalPose, size_t numNodes, size_t numThreads) =
    // 0;
    // virtual bool computePathToPose(const Vector<dim> startConfig, const std::vector<Vector6> pathPoses, size_t numNodes, size_t
    // numThreads) = 0;
    virtual bool expand(size_t numNode, size_t numthreads) = 0;

    std::shared_ptr<Graph<dim>> getGraph();
    std::vector<std::shared_ptr<Node<dim>>> getGraphNodes();
    virtual std::vector<Vector<dim>> getPath(double posRes = 1, double oriRes = 0.1) = 0;
    virtual std::vector<std::shared_ptr<Node<dim>>> getPathNodes() = 0;
    std::vector<Vector<dim>> getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes, double posRes, double oriRes);

  protected:
      void setSamplingParams(const Vector<dim> &start, const Vector<dim> &goal);

    std::shared_ptr<ValidityChecker<dim>> m_validityChecker = nullptr;
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<DistanceMetric<dim>> m_metric = nullptr;
    std::shared_ptr<Evaluator<dim>> m_evaluator = nullptr;
    std::shared_ptr<PathModifier<dim>> m_pathModifier = nullptr;
    std::shared_ptr<Graph<dim>> m_graph = nullptr;
    std::shared_ptr<Sampling<dim>> m_sampling = nullptr;
    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory = nullptr;

    const PlannerOptions<dim> m_options;
    bool m_pathPlanned = false; /*!< true flag, if a plan could be computed */
};

/*!
*  \brief      Standard destructor of the Planner
*  \author     Sascha Kaden
*  \date       2016-12-23
*/
template <unsigned int dim>
Planner<dim>::~Planner() {
}

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \param[in]  planner options
*  \date       2016-05-27
*/
template <unsigned int dim>
Planner<dim>::Planner(const std::string &name, const std::shared_ptr<Environment> &environment,
                      const PlannerOptions<dim> &options, const std::shared_ptr<Graph<dim>> &graph)
    : Identifier(name),
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

    // check dimensions of the robot to the dimension of the planner
    if (!util::checkDimensions<dim>(environment))
        Logging::error("Robot dimensions are unequal to planner dimension", this);
    assert(util::checkDimensions<dim>(environment));
}

/*!
*  \brief      Return the graph of the path planner
*  \author     Sascha Kaden
*  \param[out] Graph
*  \date       2016-09-27
*/
template <unsigned int dim>
std::shared_ptr<Graph<dim>> Planner<dim>::getGraph() {
    return m_graph;
}

/*!
*  \brief      Return all nodes from the Graph
*  \author     Sascha Kaden
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Planner<dim>::getGraphNodes() {
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
std::vector<Vector<dim>> Planner<dim>::getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes, double posRes,
                                                        double oriRes) {
    std::vector<std::shared_ptr<Node<dim>>> smoothedNodes = m_pathModifier->smoothPath(nodes);

    Logging::info("Path has after smoothing: " + std::to_string(smoothedNodes.size()) + " nodes", this);

    // save the planner resolution and set passed resolutions
    auto plannerRes = m_trajectory->getResolutions();
    m_trajectory->setResolutions(posRes, oriRes);
    std::vector<Vector<dim>> path;
    for (size_t i = 0; i < smoothedNodes.size() - 1; ++i) {
        path.push_back(smoothedNodes[i]->getValues());
        for (auto &config : m_trajectory->calcTrajCont(*(smoothedNodes[i]), *(smoothedNodes[i + 1])))
            path.push_back(config);
    }
    path.push_back(smoothedNodes.back()->getValues());

    // set the resolution again to the planner resolution
    m_trajectory->setResolutions(plannerRes);

    return path;
}

/*!
*  \brief      Set the Sampling and Sampler params to the used modules
*  \author     Sascha Kaden
*  \param[in]  origin
*  \param[in]  optimal path cost
*  \date       2018-03-04
*/
template <unsigned int dim>
void Planner<dim>::setSamplingParams(const Vector<dim> &start, const Vector<dim> &goal) {
    auto sampler = m_sampling->getSampler();
    sampler->setOrigin(start);
    sampler->setOptimalPathCost(m_metric->calcDist(start, goal));
}

} /* namespace ippp */

#endif /* PLANNER_HPP */
