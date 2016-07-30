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

#include <pathPlanner/Planner.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class Planner
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  TrajectoryMethod
*  \param[in]  SamplingMethod
*  \date       2016-05-27
*/
Planner::Planner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize,
                 TrajectoryMethod trajectory, SamplingMethod sampling)
    : Base(name) {
    m_pathPlanned = false;
    m_stepSize = stepSize;

    m_robot = robot;
    m_graph = std::shared_ptr<Graph>(new Graph());
    m_sampler = std::shared_ptr<Sampling>(new Sampling(m_robot, sampling));
    m_collision = std::shared_ptr<CollisionDetection>(new CollisionDetection(m_robot));
    m_planner = std::shared_ptr<TrajectoryPlanner>(new TrajectoryPlanner(trajectory, trajectoryStepSize, m_collision));
}

/*!
*  \brief      Return all nodes from the Graph
*  \author     Sascha Kaden
*  \param[out] list of all nodes
*  \date       2016-05-27
*/
std::vector<std::shared_ptr<Node>> Planner::getGraphNodes() {
    return m_graph->getNodes();
}

std::vector<Vec<float>> getPathfromNodes(std::vector<std::shared_ptr<Node>> &nodes, float trajectoryStepSize) {
    nodes = smoothPath(nodes);
    
    std::vector<Vec<float>> path;
    for (int i = 0; i < nodes.size() - 1; ++i) {
        std::vector<Vec<float>> tempVecs =
            m_planner->computeTrajectory(temp->getVec(), temp->getParent()->getVec(), trajectoryStepSize);
        for (auto vec : tempVecs)
            path.push_back(vec);
    }
    
    return path;
}

std::vector<std::shared_ptr<Node>> Planner::smoothPath(std::vector<std::shared_ptr<Node>> &nodes) {
    int i = 0;
    while (i < nodes.size()) {
        for (int j = i + 2; j < nodes.size(); ++j) {
            if (m_planner.checkTrajectory(nodes[i], nodes[j])) {
                nodes.remove(nodes.begin(), i + 1);
            }  
        }
      ++i;
    }
    return nodes;
}  
