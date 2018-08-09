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

#include <ippp/environment/Environment.h>

#include <ippp/util/Logging.h>
#include <ippp/environment/cad/CadImportExport.h>

namespace ippp {

/*!
*  \brief      Constructor of the class Environment
*  \author     Sascha Kaden
*  \param[in]  dimension of the workspace
*  \param[in]  workspace boundary
*  \date       2017-05-17
*/
Environment::Environment(const AABB &spaceBoundary) : Identifier("Environment"), m_spaceBoundary(spaceBoundary) {
    Logging::debug("Initialize", this);
}

/*!
*  \brief      Constructor of the class Environment
*  \author     Sascha Kaden
*  \param[in]  dimension of the workspace
*  \param[in]  workspace boundary
*  \param[in]  robot
*  \date       2017-05-17
*/
Environment::Environment(const AABB &spaceBoundary, const std::shared_ptr<RobotBase> &robot)
    : Identifier("Environment"), m_spaceBoundary(spaceBoundary) {
    Logging::debug("Initialize", this);

    addRobot(robot);
}

/*!
*  \brief      Constructor of the class Environment
*  \author     Sascha Kaden
*  \param[in]  dimension of the workspace
*  \param[in]  workspace boundary
*  \param[in]  list of robots
*  \date       2017-05-17
*/
Environment::Environment(const AABB &spaceBoundary, const std::vector<std::shared_ptr<RobotBase>> &robots)
    : Identifier("Environment"), m_spaceBoundary(spaceBoundary) {
    Logging::debug("Initialize", this);

    addRobot(robots);
}

Environment::~Environment() = default;

bool Environment::addObstacle(const std::shared_ptr<ObstacleObject> &obstacle) {
    if (!obstacle) {
        Logging::error("Empty obstacle object!", this);
        return false;
    }
    m_obstacles.push_back(obstacle);
    update();
    return true;
}

bool Environment::addObstacle(const std::vector<std::shared_ptr<ObstacleObject>> &obstacles) {
    if (obstacles.empty()) {
        Logging::error("Empty obstacle list!", this);
        return false;
    }

    for (auto &obstacle : obstacles)
        if (!addObstacle(obstacle))
            return false;

    return true;
}

bool Environment::addRobot(const std::shared_ptr<RobotBase> &robot) {
    if (!robot) {
        Logging::error("Empty robot object!", this);
        return false;
    }
    m_robots.push_back(robot);
    update();
    return true;
}

bool Environment::addRobot(const std::vector<std::shared_ptr<RobotBase>> &robots) {
    if (robots.empty()) {
        Logging::error("Empty robot list!", this);
        return false;
    }

    for (auto &robot : robots)
        if (!addRobot(robot))
            return false;

    return true;
}

/*!
*  \brief      Return number of EnvObjects of the Environment
*  \author     Sascha Kaden
*  \param[out] EnvObject size
*  \date       2018-01-10
*/
size_t Environment::numObjects() const {
    return m_obstacles.size() + m_robots.size();
}

/*!
*  \brief      Return number of obstacles of the Environment
*  \author     Sascha Kaden
*  \param[out] obstacle size
*  \date       2018-01-10
*/
size_t Environment::numObstacles() const {
    return m_obstacles.size();
}

/*!
*  \brief      Return number of robots of the Environment
*  \author     Sascha Kaden
*  \param[out] robot size
*  \date       2018-01-10
*/
size_t Environment::numRobots() const {
    return m_robots.size();
}

/*!
*  \brief      Return vector of ObstacleObjects of the Environment.
*  \author     Sascha Kaden
*  \param[out] vector of ObstacleObject
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<ObstacleObject>> Environment::getObstacles() const {
    return m_obstacles;
}

/*!
*  \brief      Return the robot by index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] RobotBase
*  \date       2018-01-10
*/
std::shared_ptr<ObstacleObject> Environment::getObstacle(size_t index) const {
    if (m_obstacles.size() <= index) {
        Logging::error("Obstacle index to large!", this);
        return nullptr;
    }
    return m_obstacles[index];
}

/*!
*  \brief      Return vector of robots of the Environment.
*  \author     Sascha Kaden
*  \param[out] vector of shared_ptr of RobotBase
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<RobotBase>> Environment::getRobots() const {
    return m_robots;
}

/*!
*  \brief      Return the first robot
*  \author     Sascha Kaden
*  \param[out] RobotBase
*  \date       2018-01-10
*/
std::shared_ptr<RobotBase> Environment::getRobot() const {
    if (m_robots.empty()) {
        Logging::error("No robot is set!", this);
        return nullptr;
    }
    return m_robots.front();
}

/*!
*  \brief      Return the robot by index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] RobotBase
*  \date       2018-01-10
*/
std::shared_ptr<RobotBase> Environment::getRobot(size_t index) const {
    if (m_robots.size() <= index) {
        Logging::error("Robot index to large!", this);
        return nullptr;
    }
    return m_robots[index];
}

/*!
*  \brief      Return bounding box of the Environment
*  \author     Sascha Kaden
*  \param[out] bounding box
*  \date       2017-05-17
*/
AABB Environment::getSpaceBoundary() const {
    return m_spaceBoundary;
}

/*!
*  \brief      Return the list of the dimension sizes of the robots.
*  \author     Sascha Kaden
*  \param[out] robot dimension sizes
*  \date       2017-05-17
*/
std::vector<unsigned int> Environment::getRobotDimSizes() const {
    std::vector<unsigned int> robotDimSizes;
    for (const auto &robot : getRobots())
        robotDimSizes.push_back(robot->getDim());
    return robotDimSizes;
}

/*!
*  \brief      Return the configuration dimension.
*  \author     Sascha Kaden
*  \param[out] configuration dimension
*  \date       2017-09-30
*/
unsigned int Environment::getConfigDim() const {
    return m_configDim;
}

/*!
*  \brief      Return the boundaries (minimum and maximum) of all robots in one Vector
*  \author     Sascha Kaden
*  \param[out] pair of a min and max Vector
*  \date       2017-11-14
*/
std::pair<VectorX, VectorX> Environment::getRobotBoundaries() const {
    return m_robotBoundaries;
}

/*!
*  \brief      Return the position and rotation mask of the robots.
*  \author     Sascha Kaden
*  \param[out] position and rotation mask
*  \date       2017-09-30
*/
std::pair<VectorX, VectorX> Environment::getConfigMasks() const {
    return std::make_pair(m_positionMask, m_rotationMask);
}

void Environment::saveObstacleMeshes(const std::string &folder) const {
    size_t count = 0;
    for (auto &obstacle : m_obstacles) {
        std::vector<Vector3> vertices = obstacle->model->m_mesh.vertices;
        cad::transformVertices(obstacle->getPose(), vertices);
        cad::exportCad(cad::ExportFormat::OBJ, folder + "obs" + std::to_string(count), vertices, obstacle->model->m_mesh.faces);
        ++count;
    }
}

void Environment::update() {
    m_configDim = 0;
    for (const auto &robot : m_robots)
        m_configDim += robot->getDim();

    updateMasks();
    updateRobotBoundaries();
}

/*!
*  \brief      Update the position and rotation mask of the robots inside of the environment.
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
void Environment::updateMasks() {
    m_positionMask = VectorX(m_configDim, 1);
    m_rotationMask = VectorX(m_configDim, 1);
    size_t index = 0;
    for (auto &robot : m_robots) {
        auto dofTypes = robot->getDofTypes();
        for (auto dof = dofTypes.begin(); dof != dofTypes.end(); ++dof, ++index) {
            if (*dof == DofType::planarPos || *dof == DofType::volumetricPos || *dof == DofType::jointTrans) {
                m_positionMask[index] = 1;
                m_rotationMask[index] = 0;
            } else {
                m_positionMask[index] = 0;
                m_rotationMask[index] = 1;
            }
        }
    }
}

void Environment::updateRobotBoundaries() {
    m_robotBoundaries.first = VectorX(m_configDim, 1);
    m_robotBoundaries.second = VectorX(m_configDim, 1);

    size_t index = 0;
    for (auto &robot : m_robots) {
        auto boundary = robot->getBoundary();
        for (unsigned int dim = 0; dim < robot->getDim(); ++dim, ++index) {
            m_robotBoundaries.first[index] = boundary.first[dim];
            m_robotBoundaries.second[index] = boundary.second[dim];
        }
    }
}

} /* namespace ippp */
