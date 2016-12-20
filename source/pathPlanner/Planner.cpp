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

#include <include/core/utility/Logging.h>
#include <pathPlanner/Planner.h>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \param[in]  planner options
*  \date       2016-05-27
*/
Planner::Planner(const std::string &name, const shared_ptr<RobotBase> &robot, const PlannerOptions &options)
    : ModuleBase(name) {
    m_pathPlanned = false;

    m_robot = robot;
    m_graph = shared_ptr<Graph>(new Graph());
    m_sampler = shared_ptr<Sampling>(new Sampling(m_robot, options.getSamplingMethod(), options.getSamplingStrategy()));
    m_collision = shared_ptr<CollisionDetection>(new CollisionDetection(m_robot));
    m_planner = shared_ptr<TrajectoryPlanner>(new TrajectoryPlanner(options.getTrajectoryStepSize(), m_collision));
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
*  \param[in]
*  \param[in]
*  \param[in]
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
std::vector<Vec<float>> Planner::getPathFromNodes(const std::vector<shared_ptr<Node>> &nodes, float trajectoryStepSize,
                                                  bool smoothing) {
    std::vector<shared_ptr<Node>> smoothedNodes;
    if (smoothing)
        smoothedNodes = smoothPath(nodes);
    else
        smoothedNodes = nodes;

    Logging::info("Path has after smoothing: " + std::to_string(smoothedNodes.size()) + " nodes", this);

    std::vector<Vec<float>> path;
    for (int i = 0; i < smoothedNodes.size() - 1; ++i) {
        std::vector<Vec<float>> tempVecs =
            m_planner->computeTrajectory(smoothedNodes[i]->getVec(), smoothedNodes[i + 1]->getVec(), trajectoryStepSize);
        for (auto vec : tempVecs)
            path.push_back(vec);
    }
    return path;
}

std::vector<shared_ptr<Node>> Planner::smoothPath(std::vector<shared_ptr<Node>> nodes) {
    unsigned int i = 0;
    unsigned int countNodes = nodes.size() - 2;
    while (i < countNodes) {
        while (i < countNodes && m_planner->controlTrajectory(nodes[i]->getVec(), nodes[i + 2]->getVec())) {
            nodes.erase(nodes.begin() + i + 1);
            --countNodes;
        }
        ++i;
    }
    return nodes;
}

} /* namespace rmpl */