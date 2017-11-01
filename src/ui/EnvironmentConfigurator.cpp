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

#include <ippp/ui/EnvironmentConfigurator.h>

namespace ippp {

/*!
*  \brief      Constructor of the class EnvironmentConfigurator
*  \author     Sascha Kaden
*  \date       2017-10-16
*/
EnvironmentConfigurator::EnvironmentConfigurator() : Configurator("EnvironmentConfigurator") {
}

/*!
*  \brief      Saves the environment configuration into a json file.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] result of the saving
*  \date       2017-10-18
*/
bool EnvironmentConfigurator::saveConfig(const std::string &filePath) {
    nlohmann::json json;

    json["NumObstacles"] = m_obstaclePaths.size();
    for (size_t i = 0; i < m_obstaclePaths.size(); ++i)
        json["ObstaclePath" + std::to_string(i)] = m_obstaclePaths[i];
    json["WorkspaceDim"] = m_workspaceDim;
    Vector3 bottomLeft = m_workspceBounding.corner(AABB::CornerType::BottomLeft);
    Vector3 topRight = m_workspceBounding.corner(AABB::CornerType::TopRight);
    json["MinWorkspaceBound"] = vectorToString<3>(bottomLeft);
    json["MaxWorkspaceBound"] = vectorToString<3>(topRight);
    json["FactoryType"] = static_cast<int>(m_factoryType);
    json["RobotType"] = static_cast<int>(m_robotType);

    return saveJson(filePath, json);
}

/*!
*  \brief      Loads the environment configuration from a json file.
*  \author     Sascha Kaden
*  \param[in]  file path
*  \param[out] result of the loading
*  \date       2017-10-18
*/
bool EnvironmentConfigurator::loadConfig(const std::string &filePath) {
    nlohmann::json json = loadJson(filePath);
    if (json.empty())
        return false;

    m_obstaclePaths.clear();
    size_t numObstacles = json["NumObstacles"].get<size_t>();
    for (size_t i = 0; i < numObstacles; ++i)
        m_obstaclePaths.push_back(json["ObstaclePath" + std::to_string(i)].get<std::string>());
    m_workspaceDim = json["WorkspaceDim"].get<unsigned int>();
    Vector3 bottomLeft = stringToVector<3>(json["MinWorkspaceBound"].get<std::string>());
    Vector3 topRight = stringToVector<3>(json["MaxWorkspaceBound"].get<std::string>());
    m_workspceBounding = AABB(bottomLeft, topRight);
    m_factoryType = static_cast<FactoryType>(json["FactoryType"].get<int>());
    m_robotType = static_cast<RobotType>(json["RobotType"].get<int>());

    return true;
}

/*!
*  \brief      Sets the properties of the workspace of the environment.
*  \author     Sascha Kaden
*  \param[in]  workspace dimension
*  \param[in]  AABB of the workspace bounding
*  \date       2017-10-18
*/
void EnvironmentConfigurator::setWorkspaceProperties(const unsigned int workspaceDim, const AABB &workspaceBounding) {
    m_workspaceDim = workspaceDim;
    m_workspceBounding = workspaceBounding;
}

/*!
*  \brief      Set the paths to the mesh files of the obstacles.
*  \author     Sascha Kaden
*  \param[in]  obstacle file paths
*  \date       2017-10-18
*/
void EnvironmentConfigurator::setObstaclePaths(const std::vector<std::string> &obstaclePaths) {
    if (!obstaclePaths.empty())
        m_obstaclePaths = obstaclePaths;
}

/*!
*  \brief      Adds a path of one obstacle mesh file.
*  \author     Sascha Kaden
*  \param[in]  obstacle file paths
*  \date       2017-10-18
*/
void EnvironmentConfigurator::addObstaclePath(const std::string &obstaclePath) {
    if (!obstaclePath.empty())
        m_obstaclePaths.push_back(obstaclePath);
}

/*!
*  \brief      Sets the FactoryType of the ModelFactory
*  \author     Sascha Kaden
*  \param[in]  FactoryType
*  \date       2017-10-18
*/
void EnvironmentConfigurator::setFactoryType(const FactoryType factoryType) {
    m_factoryType = factoryType;
}

/*!
*  \brief      Sets the RobotType and the file path to the mesh file of the robot.
*  \author     Sascha Kaden
*  \param[in]  RobotType
*  \param[in]  robot mesh file
*  \date       2017-10-18
*/
void EnvironmentConfigurator::setRobotType(const RobotType robotType, const std::string &robotFile) {
    m_robotType = robotType;

    if (robotFile.empty())
        return;

    m_robotFile = robotFile;
}

/*!
*  \brief      Returns the created Environment with the specified options.
*  \author     Sascha Kaden
*  \param[out] Environment
*  \date       2017-10-18
*/
std::shared_ptr<Environment> EnvironmentConfigurator::getEnvironment() {
    m_environment = std::shared_ptr<Environment>(new Environment(m_workspaceDim, m_workspceBounding));

    std::shared_ptr<ModelFactory> factory;
    switch (m_factoryType) {
        case ippp::FactoryType::ModelFCL:
            factory = std::shared_ptr<ModelFactory>(new ModelFactoryFcl);
            break;
        case ippp::FactoryType::ModelPQP:
            factory = std::shared_ptr<ModelFactory>(new ModelFactoryPqp);
            break;
        case ippp::FactoryType::ModelTriangle2D:
            factory = std::shared_ptr<ModelFactory>(new ModelFactoryTriangle2D);
            break;
        default:
            factory = std::shared_ptr<ModelFactory>(new ModelFactoryPqp);
            break;
    }

    for (auto &obstaclePath : m_obstaclePaths)
        m_environment->addObstacle(factory->createModel(obstaclePath));

    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();
    m_robot = nullptr;
    if (m_robotType == RobotType::Point) {
        Vector2 tempMin, tempMax;
        for (size_t i = 0; i < 2; ++i) {
            tempMin[i] = min[i];
            tempMax[i] = max[i];
        }
        m_robot = std::shared_ptr<RobotBase>(new PointRobot(std::make_pair(tempMin, tempMax)));
    } else if (m_robotType == RobotType::Serial2D) {
        m_robot = std::shared_ptr<RobotBase>(new SerialRobot2D);
    } else if (m_robotType == RobotType::Mobile) {
        Vector6 min6, max6;
        min6 = util::Vecd(min[0], min[1], min[2], 0, 0, 0);
        max6 = util::Vecd(max[0], max[1], max[2], util::twoPi(), util::twoPi(), util::twoPi());
        std::vector<DofType> types = {volumetricPos, volumetricPos, volumetricPos, volumetricRot, volumetricRot, volumetricRot};
        auto robotModel = factory->createModel(m_robotFile);
        m_robot = std::shared_ptr<RobotBase>(new MobileRobot(6, std::make_pair(min6, max6), types));
        m_robot->setBaseModel(robotModel);
    } else if (m_robotType == RobotType::Triangle2D) {
        Vector3 tempMin, tempMax;
        for (size_t i = 0; i < 2; ++i) {
            tempMin[i] = min[i];
            tempMax[i] = max[i];
        }
        tempMin[2] = 0;
        tempMax[2] = 360 * util::toRad();
        auto robotModel = factory->createModel(m_robotFile);
        m_robot = std::shared_ptr<RobotBase>(new TriangleRobot2D(robotModel, std::make_pair(tempMin, tempMax)));
    }

    if (m_robot)
        m_environment->addRobot(m_robot);

    return m_environment;
}

} /* namespace ippp */
