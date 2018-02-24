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

#ifndef ENVIRONMENTCONFIGURATOR_H
#define ENVIRONMENTCONFIGURATOR_H

#include <type_traits>
#include <vector>

#include <ippp/Environment.h>
#include <ippp/types.h>
#include <ippp/ui/Configurator.h>

namespace ippp {

enum class FactoryType { ModelFCL, ModelPQP, ModelTriangle2D };

enum class RobotType { Jaco, Point, Triangle2D, Serial, Serial2D, Mobile };

/*!
* \brief   Class EnvironmentConfigurator constructs the environment of the planner
* \author  Sascha Kaden
* \date    2017-10-16
*/
class EnvironmentConfigurator : public Configurator {
  public:
    EnvironmentConfigurator();
    bool saveConfig(const std::string &filePath);
    bool loadConfig(const std::string &filePath);

    void setWorkspaceProperties(const AABB &workspaceBounding);
    void addObstacle(const std::string &obstaclePath, const Vector6 &pose = Vector6::Zero());
    void setFactoryType(const FactoryType factoryType);
    void setRobotType(const RobotType robotType);
    void setRobotBaseModelFile(const std::string robotBaseModelFile);
    void setRobotBaseProperties(size_t robotDim, const std::vector<DofType> &dofTypes,
                                const std::pair<VectorX, VectorX> &robotBoundaries,
                                const Transform &pose = Transform::Identity());
    void setSerialRobotProperties(const std::vector<DhParameter> &dhParameters, const std::vector<std::string> &linkModelFiles,
                                  const std::vector<Transform> &linkOffsets = std::vector<Transform>(),
                                  const Transform &baseOffset = Transform::Identity(),
                                  const Transform &toolOffset = Transform::Identity(), std::string toolModelFile = "");

    std::shared_ptr<Environment> getEnvironment();
    std::string getRobotBaseModelFile() const;
    std::vector<std::string> getLinkModelFiles() const;
    std::vector<std::string> getObstaclePaths() const;
    std::vector<Transform> getObstaclePoses() const;

  protected:
    std::shared_ptr<RobotBase> createPointRobot();
    std::shared_ptr<RobotBase> createTriangleRobot(ModelFactory &factory);
    std::shared_ptr<RobotBase> createMobileRobot(ModelFactory &factory);
    std::shared_ptr<RobotBase> createSerialRobot(ModelFactory &factory, RobotType type = RobotType::Serial);

    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<RobotBase> m_robot = nullptr;

    std::vector<std::string> m_obstaclePaths;
    std::vector<Transform> m_obstacleTransforms;
    AABB m_workspceBounding;

    FactoryType m_factoryType = FactoryType::ModelTriangle2D;
    RobotType m_robotType = RobotType::Point;

    // robot base properties
    size_t m_robotDim;
    std::vector<DofType> m_dofTypes;
    std::pair<VectorX, VectorX> m_robotBoundaries;
    std::string m_robotBaseModelFile;
    Transform m_robotPose = Transform::Identity();

    // serial robot properties
    std::vector<DhParameter> m_dhParameters;
    std::vector<std::string> m_linkModelFiles;
    std::vector<Transform> m_linkOffsets;
    Transform m_baseOffset = Transform::Identity();
    Transform m_toolOffset = Transform::Identity();
    std::string m_toolModelFile;
};

} /* namespace ippp */

#endif    // ENVIRONMENTCONFIGURATOR_H
