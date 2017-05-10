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

#include <core/dataObj/Graph.hpp>
#include <core/module/Identifier.h>
#include <core/module/TrajectoryPlanner.hpp>
#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <core/module/sampling/Sampling.hpp>
#include <core/types.h>
#include <pathPlanner/options/PlannerOptions.hpp>
#include <robot/RobotBase.hpp>

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
    Planner(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot, const PlannerOptions<dim> &options);

  public:
    virtual bool computePath(const Vector<dim> start, const Vector<dim> goal, const unsigned int numNodes,
                             const unsigned int numThreads) = 0;
    virtual bool expand(const unsigned int numNode, const unsigned int numthreads) = 0;

    std::shared_ptr<Graph<dim>> getGraph();
    std::vector<std::shared_ptr<Node<dim>>> getGraphNodes();
    virtual std::vector<Vector<dim>> getPath(const float trajectoryStepSize, const bool smoothing) = 0;
    virtual std::vector<std::shared_ptr<Node<dim>>> getPathNodes() = 0;
    std::vector<Vector<dim>> getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes,
                                              const float trajectoryStepSize, const bool smoothing);

  protected:
    std::vector<std::shared_ptr<Node<dim>>> smoothPath(std::vector<std::shared_ptr<Node<dim>>> nodes);

    std::shared_ptr<TrajectoryPlanner<dim>> m_trajectory;
    std::shared_ptr<Sampling<dim>> m_sampling;
    std::shared_ptr<CollisionDetection<dim>> m_collision;
    std::shared_ptr<Graph<dim>> m_graph;
    std::shared_ptr<RobotBase<dim>> m_robot;

    const std::shared_ptr<Heuristic<dim>> m_heuristic;
    const PlannerOptions<dim> m_options;
    bool m_pathPlanned;
};

/*!
*  \brief      Standard deconstructor of the Planner
*  \author     Sasch Kaden
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
Planner<dim>::Planner(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot, const PlannerOptions<dim> &options)
    : Identifier(name), m_options(options), m_heuristic(options.getHeuristic()) {
    m_pathPlanned = false;

    m_robot = robot;
    m_graph = std::shared_ptr<Graph<dim>>(new Graph<dim>(options.getSortCountGraph()));
    m_collision = m_options.getCollisionDetection();
    m_trajectory = options.getTrajectoryPlanner();
    m_sampling = options.getSampling();
}

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
*  \param[in]  smoothing
*  \param[out] path configurations
*  \date       2016-05-27
*/
template <unsigned int dim>
std::vector<Vector<dim>> Planner<dim>::getPathFromNodes(const std::vector<std::shared_ptr<Node<dim>>> &nodes,
                                                        const float trajectoryStepSize, const bool smoothing) {
    std::vector<std::shared_ptr<Node<dim>>> smoothedNodes;
    if (smoothing)
        smoothedNodes = smoothPath(nodes);
    else
        smoothedNodes = nodes;

    Logging::info("Path has after smoothing: " + std::to_string(smoothedNodes.size()) + " nodes", this);

    std::vector<Vector<dim>> path;
    for (int i = 0; i < smoothedNodes.size() - 1; ++i) {
        std::vector<Vector<dim>> tempVecs =
            m_trajectory->calcTrajectoryCont(smoothedNodes[i]->getValues(), smoothedNodes[i + 1]->getValues());
        for (auto vec : tempVecs) {
            path.push_back(vec);
        }
    }

    if (trajectoryStepSize < m_trajectory->getStepSize()) {
        Logging::info("Passed trajectory step size is smaller than the step of the planner! Path has step size of the planner",
                      this);
    } else if (trajectoryStepSize > m_trajectory->getStepSize()) {
        unsigned int steps = trajectoryStepSize / m_trajectory->getStepSize();
        std::vector<Vector<dim>> newPath;
        newPath.push_back(path[0]);
        unsigned int j = 0;
        for (auto point : path) {
            if (j == steps) {
                newPath.push_back(point);
                j = 0;
            } else {
                ++j;
            }
        }

        if (newPath.back() != path.back()) {
            newPath.push_back(path.back());
        }
        path = newPath;
    }
    return path;
}

/*!
*  \brief      Return shortened path nodes
*  \details    If trajectory from node to grandparent node is free, parent node will be erased.
*  \author     Sascha Kaden
*  \param[in]  path nodes
*  \param[out] shorted path nodes
*  \date       2016-05-27
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Planner<dim>::smoothPath(std::vector<std::shared_ptr<Node<dim>>> nodes) {
    unsigned int i = 0;
    auto countNodes = nodes.size() - 2;
    while (i < countNodes) {
        while (i < countNodes && m_trajectory->controlTrajectory(nodes[i]->getValues(), nodes[i + 2]->getValues())) {
            nodes.erase(nodes.begin() + i + 1);
            --countNodes;
        }
        ++i;
    }
    return nodes;
}

} /* namespace ippp */

#endif /* RRTPLANNER_HPP */
