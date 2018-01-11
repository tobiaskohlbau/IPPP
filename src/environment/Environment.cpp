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

#include <ippp/environment/Environment.h>

#include <ippp/util/Logging.h>

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
Environment::Environment(const AABB &spaceBoundary, const std::shared_ptr<EnvObject> &object)
    : Identifier("Environment"), m_spaceBoundary(spaceBoundary) {
    Logging::debug("Initialize", this);

    addEnvObject(object);
}

/*!
*  \brief      Constructor of the class Environment
*  \author     Sascha Kaden
*  \param[in]  dimension of the workspace
*  \param[in]  workspace boundary
*  \param[in]  list of robots
*  \date       2017-05-17
*/
Environment::Environment(const AABB &spaceBoundary, const std::vector<std::shared_ptr<EnvObject>> &objects)
    : Identifier("Environment"), m_spaceBoundary(spaceBoundary) {
    Logging::debug("Initialize", this);

    addEnvObjects(objects);
}

Environment::~Environment() = default;

/*!
*  \brief      Add an EnvObject to the Environment and update the robot parameter.
*  \author     Sascha Kaden
*  \param[in]  EnvObject
*  \date       2018-01-10
*/
void Environment::addEnvObject(const std::shared_ptr<EnvObject> &object) {
    if (!object) {
        Logging::error("Empty environment object", this);
        return;
    }

    m_envObjects.push_back(object);
    update();
}

/*!
*  \brief      Add EnvObjects to the Environment and update the robot parameter.
*  \author     Sascha Kaden
*  \param[in]  vector of EnvObject
*  \date       2018-01-10
*/
void Environment::addEnvObjects(const std::vector<std::shared_ptr<EnvObject>> &objects) {
    if (objects.empty()) {
        Logging::error("Emtpy object vector passed", this);
        return;
    }

    for (auto &object : objects) {
        if (!object) {
            Logging::error("Empty environment object", this);
            return;
        }
    }

    m_envObjects.insert(std::end(m_envObjects), std::begin(objects), std::end(objects));
    update();
}

/*!
*  \brief      Return the EnvObjects to the passed index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] EnvObject
*  \date       2018-01-10
*/
std::shared_ptr<EnvObject> Environment::getObject(size_t index) const {
    if (index >= m_envObjects.size()) {
        Logging::error("Index to large of the environment objects", this);
        return nullptr;
    }
    return m_envObjects[index];
}

/*!
*  \brief      Return vector of EnvObjects of the Environment.
*  \author     Sascha Kaden
*  \param[out] EnvObjects
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<EnvObject>> Environment::getObjects() const {
    return m_envObjects;
}

/*!
*  \brief      Return number of EnvObjects of the Environment
*  \author     Sascha Kaden
*  \param[out] EnvObject size
*  \date       2018-01-10
*/
size_t Environment::numObjects() const {
    return m_envObjects.size();
}

/*!
*  \brief      Return vector of ObstacleObjects of the Environment.
*  \author     Sascha Kaden
*  \param[out] vector of ObstacleObject
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<ObstacleObject>> Environment::getObstacles() const {
    std::vector<std::shared_ptr<ObstacleObject>> obstacles;
    for (auto &object : m_envObjects) {
        if (object->m_type == EnvObjectType::Obstacle) {
            obstacles.push_back(std::dynamic_pointer_cast<ObstacleObject>(object));
        }
    }
    return obstacles;
}

/*!
*  \brief      Return number of obstacles of the Environment
*  \author     Sascha Kaden
*  \param[out] obstacle size
*  \date       2018-01-10
*/
size_t Environment::numObstacles() const {
    return getObstacles().size();
}

/*!
*  \brief      Return vector of robots of the Environment.
*  \author     Sascha Kaden
*  \param[out] vector of shared_ptr of RobotBase
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<RobotBase>> Environment::getRobots() const {
    std::vector<std::shared_ptr<RobotBase>> robots;
    for (auto &object : m_envObjects) {
        if (object->m_type == EnvObjectType::Robot) {
            robots.push_back(std::dynamic_pointer_cast<RobotBase>(object));
        }
    }
    return robots;
}

/*!
*  \brief      Return the first robot
*  \author     Sascha Kaden
*  \param[out] RobotBase
*  \date       2018-01-10
*/
std::shared_ptr<RobotBase> Environment::getRobot() const {
    auto robots = getRobots();
    if (robots.size() > 0)
        return robots.front();

    return nullptr;
}

/*!
*  \brief      Return number of robots of the Environment
*  \author     Sascha Kaden
*  \param[out] robot size
*  \date       2018-01-10
*/
size_t Environment::numRobots() const {
    return getRobots().size();
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

void Environment::update() {
    updateConfigDim();
    updateMasks();
    updateRobotBoundaries();
}

/*!
*  \brief      Update the complete configuration dimensions of all robots inside of the environment.
*  \author     Sascha Kaden
*  \date       2017-09-30
*/
void Environment::updateConfigDim() {
    m_configDim = 0;
    for (const auto &robot : getRobots())
        m_configDim += robot->getDim();

    updateMasks();
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
    for (auto &robot : getRobots()) {
        auto dofTypes = robot->getDofTypes();
        for (auto dof = dofTypes.begin(); dof != dofTypes.end(); ++dof, ++index) {
            if (*dof == DofType::planarPos || *dof == DofType::volumetricPos || *dof == DofType::position) {
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
    for (auto &robot : getRobots()) {
        auto min = robot->getMinBoundary();
        auto max = robot->getMaxBoundary();
        for (unsigned int dim = 0; dim < robot->getDim(); ++dim, ++index) {
            m_robotBoundaries.first[index] = min[dim];
            m_robotBoundaries.second[index] = max[dim];
        }
    }
}

} /* namespace ippp */
