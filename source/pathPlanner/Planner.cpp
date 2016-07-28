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
Planner::Planner(const std::string &name, const std::shared_ptr<RobotBase> &robot, float stepSize, float trajectoryStepSize, TrajectoryMethod trajectory, SamplingMethod sampling)
        : Base(name){
    m_pathPlanned = false;
    m_stepSize = stepSize;

    m_robot = robot;
    m_graph = std::shared_ptr<Graph>(new Graph());
    m_sampler = std::shared_ptr<Sampling>(new Sampling(m_robot, sampling));
    m_vrep = std::shared_ptr<Helper>(new Helper(m_robot->getDim()));
    m_collision = std::shared_ptr<CollisionDetection>(new CollisionDetection(m_vrep, m_robot));
    m_planner = std::shared_ptr<TrajectoryPlanner>(new TrajectoryPlanner(trajectory, trajectoryStepSize, m_collision));

    //if (m_robot->getDim() != 2)
    //    m_vrep->start();
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

/*!
*  \brief      Return the pointer to the VREP Helper initiation
*  \author     Sascha Kaden
*  \param[out] pointer to VREP Helper
*  \date       2016-05-27
*/
std::shared_ptr<Helper> Planner::getVrep() {
    return m_vrep;
}
