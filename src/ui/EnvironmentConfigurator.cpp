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

    // workspace
    json["ObstaclePaths"] = m_obstaclePaths;
    json["ObstacleTransforms"] = jsonSerializer::serialize(m_obstacleTransforms);
    json["WorkspaceBound"] = jsonSerializer::serialize(m_workspceBounding);
    json["FactoryType"] = static_cast<int>(m_factoryType);
    // base robot
    json["RobotType"] = static_cast<int>(m_robotType);
    json["RobotDim"] = m_robotDim;
    json["DofTypes"] = jsonSerializer::serialize(m_dofTypes);
    json["MinRobotBound"] = jsonSerializer::serialize(m_robotBoundaries.first);
    json["MaxRobotBound"] = jsonSerializer::serialize(m_robotBoundaries.second);
    json["RobotBaseModelFile"] = m_robotBaseModelFile;
    json["RobotPose"] = jsonSerializer::serialize(m_robotPose);
    // serial robot
    json["DhParams"] = jsonSerializer::serialize(m_dhParameters);
    json["LinkModelFiles"] = m_linkModelFiles;
    json["LinkOffsets"] = jsonSerializer::serialize(m_linkOffsets);
    json["BaseOffset"] = jsonSerializer::serialize(m_baseOffset);
    json["ToolOffset"] = jsonSerializer::serialize(m_toolOffset);
    json["ToolModelFile"] = m_toolModelFile;

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

    // workspace
    m_obstaclePaths = json["ObstaclePaths"].get<std::vector<std::string>>();
    if (!m_obstaclePaths.empty())
        m_obstacleTransforms = jsonSerializer::deserializeTransforms(json["ObstacleTransforms"]);
    m_workspceBounding = jsonSerializer::deserializeAABB(json["WorkspaceBound"]);
    m_factoryType = static_cast<FactoryType>(json["FactoryType"].get<int>());
    // base robot
    m_robotType = static_cast<RobotType>(json["RobotType"].get<int>());
    m_robotDim = json["RobotDim"].get<int>();
    m_dofTypes = jsonSerializer::deserializeDofTypes(json["DofTypes"]);
    m_robotBoundaries = std::make_pair(jsonSerializer::deserializeVector(json["MinRobotBound"]),
                                       jsonSerializer::deserializeVector(json["MaxRobotBound"]));
    m_robotBaseModelFile = json["RobotBaseModelFile"].get<std::string>();
    m_robotPose = jsonSerializer::deserializeTransform(json["RobotPose"]);
    // serial robot
    m_dhParameters = jsonSerializer::deserializeDhParameters(json["DhParams"]);
    m_linkModelFiles = json["LinkModelFiles"].get<std::vector<std::string>>();
    m_linkOffsets = jsonSerializer::deserializeTransforms(json["LinkOffsets"]);
    m_baseOffset = jsonSerializer::deserializeTransform(json["BaseOffset"]);
    m_toolOffset = jsonSerializer::deserializeTransform(json["ToolOffset"]);
    m_toolModelFile = json["ToolModelFile"].get<std::string>();

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
*  \brief      Adds a path of one obstacle mesh file.
*  \author     Sascha Kaden
*  \param[in]  obstacle file paths
*  \date       2017-10-18
*/
void EnvironmentConfigurator::addObstacle(const std::string &obstaclePath, const Vector6 &pose) {
    if (obstaclePath.empty()) {
        Logging::error("Empty obstacle Path", this);
        return;
    }

    m_obstaclePaths.push_back(obstaclePath);
    m_obstacleTransforms.push_back(util::toTransform(pose));
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
                                                     const std::pair<VectorX, VectorX> &robotBoundaries, const Transform &pose) {
    m_robotDim = robotDim;
    m_dofTypes = dofTypes;
    m_robotBoundaries = robotBoundaries;
    m_robotPose = pose;
}

void EnvironmentConfigurator::setRobotBaseModelFile(const std::string robotBaseModelFile) {
    if (!robotBaseModelFile.empty())
        m_robotBaseModelFile = robotBaseModelFile;
}

void EnvironmentConfigurator::setSerialRobotProperties(const std::vector<DhParameter> &dhParameters,
                                                       const std::vector<std::string> &linkModelFiles,
                                                       const std::vector<Transform> &linkOffsets, const Transform &baseOffset,
                                                       const Transform &toolOffset, std::string toolModelFile) {
    m_dhParameters = dhParameters;
    m_linkModelFiles = linkModelFiles;
    m_baseOffset = baseOffset;
    m_toolOffset = toolOffset;
    m_toolModelFile = toolModelFile;

    if (linkOffsets.empty())
        m_linkOffsets = std::vector<Transform>(m_dhParameters.size(), Transform::Identity());
    else
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

    for (size_t i = 0; i < m_obstaclePaths.size(); ++i) {
        auto model = factory->createModelFromFile(m_obstaclePaths[i]);
        auto obstacle = std::make_shared<ObstacleObject>("obstacle", model);
        obstacle->setPose(m_obstacleTransforms[i]);
        m_environment->addObstacle(obstacle);
    }

    m_robot = nullptr;
    switch (m_robotType) {
        case RobotType::Point:
            m_robot = createPointRobot();
            break;
        case RobotType::Mobile:
            m_robot = createMobileRobot(*factory);
            break;
        case RobotType::Triangle2D:
            m_robot = createTriangleRobot(*factory);
            break;
        case RobotType::Serial:
            m_robot = createSerialRobot(*factory);
            break;
        case RobotType::Jaco:
            m_robot = createSerialRobot(*factory, RobotType::Jaco);
            break;
        default:
            break;
    }
    m_robot->setPose(m_robotPose);

    if (m_robot)
        m_environment->addRobot(m_robot);

    return m_environment;
}

std::string EnvironmentConfigurator::getRobotBaseModelFile() const {
    return m_robotBaseModelFile;
}

std::vector<std::string> EnvironmentConfigurator::getLinkModelFiles() const {
    return m_linkModelFiles;
}

std::vector<std::string> EnvironmentConfigurator::getObstaclePaths() const {
    return m_obstaclePaths;
}

std::vector<Transform> EnvironmentConfigurator::getObstaclePoses() const {
    return m_obstacleTransforms;
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

std::shared_ptr<RobotBase> EnvironmentConfigurator::createTriangleRobot(ModelFactory &factory) {
    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();

    Vector3 tempMin, tempMax;
    for (size_t i = 0; i < 2; ++i) {
        tempMin[i] = min[i];
        tempMax[i] = max[i];
    }
    tempMin[2] = 0;
    tempMax[2] = 360 * util::toRad();
    auto robotModel = factory.createModelFromFile(m_robotBaseModelFile);
    return std::make_shared<TriangleRobot2D>(robotModel, std::make_pair(tempMin, tempMax));
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createMobileRobot(ModelFactory &factory) {
    Vector3 min = m_workspceBounding.min();
    Vector3 max = m_workspceBounding.max();
    Vector6 min6 = util::Vecd(min[0], min[1], min[2], 0, 0, 0);
    Vector6 max6 = util::Vecd(max[0], max[1], max[2], util::twoPi(), util::twoPi(), util::twoPi());

    std::vector<DofType> types = {volumetricPos, volumetricPos, volumetricPos, volumetricRot, volumetricRot, volumetricRot};
    auto robotModel = factory.createModelFromFile(m_robotBaseModelFile);
    auto robot = std::make_shared<MobileRobot>(6, std::make_pair(min6, max6), types);
    robot->setBaseModel(robotModel);
    return robot;
}

std::shared_ptr<RobotBase> EnvironmentConfigurator::createSerialRobot(ModelFactory &factory,
                                                                      RobotType type) {
    std::vector<Joint> joints;
    for (size_t i = 0; i < m_robotDim; ++i) {
        auto model = factory.createModelFromFile(m_linkModelFiles[i]);
        joints.push_back(
            Joint(m_robotBoundaries.first[i], m_robotBoundaries.second[i], m_dhParameters[i], model, m_linkOffsets[i]));
    }

    auto robot = std::make_shared<SerialRobot>(m_robotDim, joints, m_dofTypes);
    if (type == RobotType::Jaco)
        auto robot = std::make_shared<Jaco>(m_robotDim, joints, m_dofTypes);

    if (!m_robotBaseModelFile.empty()) {
        auto robotModel = factory.createModelFromFile(m_robotBaseModelFile);
        robot->setBaseModel(robotModel);
        robot->setBaseOffset(m_baseOffset);
    }
    
    if (!m_toolModelFile.empty()) {
        robot->setToolOffset(m_toolOffset);
        robot->setToolModel(factory.createModelFromFile(m_toolModelFile));
    }
    return robot;
}

} /* namespace ippp */
