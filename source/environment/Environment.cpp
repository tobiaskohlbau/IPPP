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

#include <environment/Environment.h>

#include <core/util/Logging.h>

namespace ippp {

/*!
*  \brief      Constructor of the class Environment
*  \author     Sascha Kaden
*  \param[in]  dimension of the workspace
*  \param[in]  workspace boundary
*  \param[in]  robot
*  \date       2017-05-17
*/
Environment::Environment(const unsigned int workspaceDim, const AABB &spaceBoundary, const std::shared_ptr<RobotBase> &robot)
    : Identifier("Environment"), m_spaceDim(workspaceDim), m_spaceBoundary(spaceBoundary) {
    if (2 > m_spaceDim || m_spaceDim > 3)
        Logging::error("Dimension of workspace have to be 2 or 3", this);
    assert(m_spaceDim == 2 || m_spaceDim == 3);
    m_robots.push_back(robot);
}

/*!
*  \brief      Standard deconstructor of the Environment
*  \author     Sasch Kaden
*  \date       2017-05-17
*/
Environment::~Environment() {
}

/*!
*  \brief      Add obstacle to the Environment
*  \author     Sascha Kaden
*  \param[in]  obstacle
*  \date       2017-05-17
*/
void Environment::addObstacle(const std::shared_ptr<ModelContainer> &model) {
    m_obstacles.push_back(model);
}

/*!
*  \brief      Return obstacle by the index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] obstacle
*  \date       2017-05-17
*/
std::shared_ptr<ModelContainer> Environment::getObstacle(const unsigned int index) const {
    if (index < m_obstacles.size()) {
        return m_obstacles[index];
    } else {
        Logging::error("Obstacel index is to high!", this);
        return nullptr;
    }
}

/*!
*  \brief      Return the list of obstacles
*  \author     Sascha Kaden
*  \param[out] list of obstacles
*  \date       2017-05-17
*/
std::vector<std::shared_ptr<ModelContainer>> Environment::getObstacles() const {
    return m_obstacles;
}

/*!
*  \brief      Return number of obstacles of the Environment
*  \author     Sascha Kaden
*  \param[out] obstacle size
*  \date       2017-05-17
*/
size_t Environment::getObstacleNum() const {
    return m_obstacles.size();
}

/*!
*  \brief      Add robot to the Environment
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-05-17
*/
void Environment::addRobot(const std::shared_ptr<RobotBase> &robot) {
    m_robots.push_back(robot);
}

/*!
*  \brief      Return first robot of the Environment
*  \author     Sascha Kaden
*  \param[out] robot
*  \date       2017-05-17
*/
std::shared_ptr<RobotBase> Environment::getRobot() const {
    if (m_robots.empty()) {
        Logging::error("No robot exists!", this);
        return nullptr;
    } else {
        return m_robots[0];
    }
}

/*!
*  \brief      Return robot by the index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] robot
*  \date       2017-05-17
*/
std::shared_ptr<RobotBase> Environment::getRobot(const unsigned int index) const {
    if (index < m_robots.size()) {
        return m_robots[index];
    } else {
        Logging::error("Robot index is to high!", this);
        return nullptr;
    }
}

/*!
*  \brief      Return the list of robots of the Environment
*  \author     Sascha Kaden
*  \param[out] list of robots
*  \date       2017-05-17
*/
std::vector<std::shared_ptr<RobotBase>> Environment::getRobots() const {
    return m_robots;
}

/*!
*  \brief      Return number of robots of the Environment
*  \author     Sascha Kaden
*  \param[out] robot size
*  \date       2017-05-17
*/
size_t Environment::getRobotNum() const {
    return m_robots.size();
}

/*!
*  \brief      Return bounding box of the Environment
*  \author     Sascha Kaden
*  \param[out] bounding box
*  \date       2017-05-17
*/
AABB Environment::getBoundary() const {
    return m_spaceBoundary;
}

/*!
*  \brief      Return dimension of the workspace
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2017-05-17
*/
unsigned int Environment::getDim() const {
    return m_spaceDim;
}

} /* namespace ippp */
