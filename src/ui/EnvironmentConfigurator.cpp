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
#include <ippp/ui/JsonSerializer.h>

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

    json["ObstaclePath"] = m_obstaclePaths;
    json["WorkspaceBound"] = jsonSerializer::serialize(m_workspceBounding);
    json["FactoryType"] = static_cast<int>(m_factoryType);
    json["RobotType"] = static_cast<int>(m_robotType);
    json["RobotDim"] = m_robotDim;
    json["DofTypes"] = jsonSerializer::serialize(m_dofTypes);
    json["MinRobotBound"] = jsonSerializer::serialize(m_robotBoundaries.first);
    json["MaxRobotBound"] = jsonSerializer::serialize(m_robotBoundaries.second);
    json["RobotBaseModelFile"] = m_robotBaseModelFile;
    json["DhParams"] = jsonSerializer::serialize(m_dhParameters);
    json["JointModelFiles"] = m_jointModelFiles;
    json["BaseOffset"] = jsonSerializer::serialize(m_baseOffset);
    json["LinkOffsets"] = jsonSerializer::serialize(m_linkOffsets);

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
    m_obstaclePaths = json["ObstaclePath"].get<std::vector<std::string>>();
    m_workspceBounding = jsonSerializer::deserializeAABB(json["WorkspaceBound"]);
    m_factoryType = static_cast<FactoryType>(json["FactoryType"].get<int>());
    m_robotType = static_cast<RobotType>(json["RobotType"].get<int>());
    m_robotDim = json["RobotDim"].get<int>();
    m_dofTypes = jsonSerializer::deserializeDofTypes(json["DofTypes"]);
    m_robotBoundaries = std::make_pair(jsonSerializer::deserializeVector(json["MinRobotBound"]),
                                       jsonSerializer::deserializeVector(json["MaxRobotBound"]));
    m_robotBaseModelFile = json["RobotBaseModelFile"].get<std::string>();
    m_dhParameters = jsonSerializer::deserializeDhParameters(json["DhParams"]);
    m_jointModelFiles = json["JointModelFiles"].get<std::vector<std::string>>();
    m_baseOffset = jsonSerializer::deserializeTransform(json["BaseOffset"]);
    m_linkOffsets = jsonSerializer::deserializeTransforms(json["LinkOffsets"]);

    return true;
}

/*!
*  \brief      Sets the properties of the workspace of the environment.
*  \author     Sascha Kaden
*  \param[in]  workspace dimension
*  \param[in]  AABB of the workspace bounding
*  \date       2017-10-18
*/
void EnvironmentConfigurator::setWorkspaceProperties(const AABB &workspaceBounding) {
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
void EnvironmentConfigurator::setRobotType(const RobotType robotType) {
    m_robotType = robotType;
}

void EnvironmentConfigurator::setRobotBaseProperties(size_t robotDim, const std::vector<DofType> &dofTypes,
                                                     const std::pair<VectorX, VectorX> &robotBoundaries) {
    m_robotDim = robotDim;
    m_dofTypes = dofTypes;
    m_robotBoundaries = robotBoundaries;
}

void EnvironmentConfigurator::setRobotBaseModelFile(const std::string robotBaseModelFile) {
    if (!robotBaseModelFile.empty())
        m_robotBaseModelFile = robotBaseModelFile;
}

void EnvironmentConfigurator::setSerialRobotProperties(const std::vector<DhParameter> &dhParameters,
                                                       const std::vector<std::string> &jointModelFiles,
                                                       const Transform &baseOffset, const std::vector<Transform> &linkOffsets) {
    m_dhParameters = dhParameters;
    m_jointModelFiles = jointModelFiles;
    m_baseOffset = baseOffset;
    m_linkOffsets = linkOffsets;
}

/*!
*  \brief      Returns the created Environment with the specified options.
*  \author     Sascha Kaden
*  \param[out] Environment
*  \date       2017-10-18
*/
std::shared_ptr<Environment> EnvironmentConfigurator::getEnvironment() {
    m_environment = std::make_shared<Environment>(m_workspceBounding);

    std::shared_ptr<ModelFactory> factory;
    switch (m_factoryType) {
        case FactoryType::ModelFCL:
            factory = std::make_shared<ModelFactoryFcl>();
            break;
        case FactoryType::ModelPQP:
            factory = std::make_shared<ModelFactoryPqp>();
            break;
        case FactoryType::ModelTriangle2D:
            factory = std::make_shared<ModelFactoryTriangle2D>();
            break;
        default:
            factory = std::make_shared<ModelFactoryFcl>();
            break;
    }

    for (auto &obstaclePath : m_obstaclePaths)
        m_environment->addObstacle(factory->createModelFromFile(obstaclePath));

    m_robot = nullptr;
    switch (m_robotType) {
        case RobotType::Point:
            m_robot = createPointRobot();
            break;
        case RobotType::Mobile:
            m_robot = createMobileRobot(factory);
            break;
        case RobotType::Triangle2D:
            m_robot = createTriangleRobot(factory);
            break;
        case RobotType::Serial:
            m_robot = createSerialRobot(factory);
            break;
        case RobotType::Jaco:
            m_robot = createSerialRobot(factory, RobotType::Jaco);
            break;
        default:
            break;
    }

    if (m_robot)
        m_environment->addRobot(m_robot);

    return m_environment;
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createPointRobot() {
    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();

    Vector2 tempMin, tempMax;
    for (size_t i = 0; i < 2; ++i) {
        tempMin[i] = min[i];
        tempMax[i] = max[i];
    }
    return std::make_shared<PointRobot>(std::make_pair(tempMin, tempMax));
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createTriangleRobot(const std::shared_ptr<ModelFactory> factory) {
    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();

    Vector3 tempMin, tempMax;
    for (size_t i = 0; i < 2; ++i) {
        tempMin[i] = min[i];
        tempMax[i] = max[i];
    }
    tempMin[2] = 0;
    tempMax[2] = 360 * util::toRad();
    auto robotModel = factory->createModelFromFile(m_robotBaseModelFile);
    return std::make_shared<TriangleRobot2D>(robotModel, std::make_pair(tempMin, tempMax));
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createMobileRobot(const std::shared_ptr<ModelFactory> factory) {
    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();

    Vector6 min6, max6;
    min6 = util::Vecd(min[0], min[1], min[2], 0, 0, 0);
    max6 = util::Vecd(max[0], max[1], max[2], util::twoPi(), util::twoPi(), util::twoPi());
    std::vector<DofType> types = {volumetricPos, volumetricPos, volumetricPos, volumetricRot, volumetricRot, volumetricRot};
    auto robotModel = factory->createModelFromFile(m_robotBaseModelFile);
    auto robot = std::make_shared<MobileRobot>(6, std::make_pair(min6, max6), types);
    robot->setBaseModel(robotModel);
    return robot;
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createSerialRobot(const std::shared_ptr<ModelFactory> factory,
                                                                      RobotType type) {
    std::vector<Joint> joints;
    for (size_t i = 0; i < m_robotDim; ++i) {
        auto model = factory->createModelFromFile(m_jointModelFiles[i]);
        joints.push_back(Joint(m_robotBoundaries.first[i], m_robotBoundaries.second[i], model));
    }

    auto robotModel = factory->createModelFromFile(m_robotBaseModelFile);

    auto robot = std::make_shared<SerialRobot>(m_robotDim, joints, m_dhParameters, m_dofTypes);
    if (type == RobotType::Jaco)
        auto robot = std::make_shared<Jaco>(m_robotDim, joints, m_dhParameters, m_dofTypes);

    robot->setBaseModel(robotModel);
    robot->setBaseOffset(m_baseOffset);
    robot->setLinkOffsets(m_linkOffsets);
    return robot;
}

} /* namespace ippp */
