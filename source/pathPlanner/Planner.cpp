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

#include <core/utility/Logging.h>
#include <pathPlanner/Planner.h>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Standard deconstructor of the Planner
*  \author     Sasch Kaden
*  \date       2016-12-23
*/
Planner::~Planner() {
}

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \param[in]  planner options
*  \date       2016-05-27
*/
Planner::Planner(const std::string &name, const shared_ptr<RobotBase> &robot, const PlannerOptions &options)
    : ModuleBase(name), m_options(options) {
    m_pathPlanned = false;

    m_robot = robot;
    m_graph = shared_ptr<Graph>(new Graph());
    m_collision = shared_ptr<CollisionDetection>(new CollisionDetection(m_robot));
    m_planner = shared_ptr<TrajectoryPlanner>(new TrajectoryPlanner(options.getTrajectoryStepSize(), m_collision));
    m_sampler = shared_ptr<Sampling>(
        new Sampling(m_robot, m_collision, m_planner, options.getSamplingMethod(), options.getSamplingStrategy()));
}

/*!
*  \brief      Return all nodes from the Graph
*  \author     Sascha Kaden
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
std::vector<shared_ptr<Node>> Planner::getGraphNodes() {
    return m_graph->getNodes();
}

/*!
*  \brief      Return path points from passed path nodes, using optional smoothing
*  \author     Sascha Kaden
*  \param[in]  path nodes
*  \param[in]  trajectory step size
*  \param[in]  smoothing
*  \param[out] path points
*  \date       2016-05-27
*/
std::vector<Eigen::VectorXf> Planner::getPathFromNodes(const std::vector<shared_ptr<Node>> &nodes, float trajectoryStepSize,
                                                       bool smoothing) {
    std::vector<shared_ptr<Node>> smoothedNodes;
    if (smoothing)
        smoothedNodes = smoothPath(nodes);
    else
        smoothedNodes = nodes;

    Logging::info("Path has after smoothing: " + std::to_string(smoothedNodes.size()) + " nodes", this);

    std::vector<Eigen::VectorXf> path;
    for (int i = 0; i < smoothedNodes.size() - 1; ++i) {
        std::vector<Eigen::VectorXf> tempVecs =
            m_planner->calcTrajectoryCont(smoothedNodes[i]->getValues(), smoothedNodes[i + 1]->getValues());
        for (auto vec : tempVecs)
            path.push_back(vec);
    }

    if (trajectoryStepSize < m_planner->getStepSize()) {
        Logging::info("Passed trajectory step size is smaller than the step of the planner! Path has step size of the planner",
                      this);
    } else if (trajectoryStepSize > m_planner->getStepSize()) {
        unsigned int steps = trajectoryStepSize / m_planner->getStepSize();
        std::vector<Eigen::VectorXf> newPath;
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

        if (newPath.back() != path.back())
            newPath.push_back(path.back());

        path = newPath;
    }
    return path;
}

/*!
*  \brief      Return shortened path nodes
*  \details    If trajectory from node to grandparent node is free, parent node will be erased.
*  \author     Sascha Kaden
*  \param[in]  path nodes
*  \param[out] shortened path nodes
*  \date       2016-05-27
*/
std::vector<shared_ptr<Node>> Planner::smoothPath(std::vector<shared_ptr<Node>> nodes) {
    unsigned int i = 0;
    unsigned int countNodes = nodes.size() - 2;
    while (i < countNodes) {
        while (i < countNodes && m_planner->controlTrajectory(nodes[i]->getValues(), nodes[i + 2]->getValues())) {
            nodes.erase(nodes.begin() + i + 1);
            --countNodes;
        }
        ++i;
    }
    return nodes;
}

} /* namespace rmpl */