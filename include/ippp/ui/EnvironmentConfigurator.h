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

enum class RobotType { Jaco, Kuka, Point, Triangle2D, Serial, Serial2D, Mobile };

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
    void setObstaclePaths(const std::vector<std::string> &obstaclePaths);
    void addObstaclePath(const std::string &obstaclePath);
    void setFactoryType(const FactoryType factoryType);
    void setRobotType(const RobotType robotType);
    void setRobotBaseModelFile(const std::string robotBaseModelFile);
    void setRobotBaseProperties(size_t robotDim, const std::vector<DofType> &dofTypes,
                                const std::pair<VectorX, VectorX> &robotBoundaries);
    void setSerialRobotProperties(const std::vector<DhParameter> &dhParameters, const std::vector<std::string> &linkModelFiles,
                                  const Transform &baseOffset = Transform::Identity(),
                                  const std::vector<Transform> &linkOffsets = std::vector<Transform>());

    std::shared_ptr<Environment> getEnvironment();
    std::string getRobotBaseModelFile() const;
    std::vector<std::string> getLinkModelFiles() const;

  protected:
    std::shared_ptr<RobotBase> createPointRobot();
    std::shared_ptr<RobotBase> createTriangleRobot(const std::shared_ptr<ModelFactory> factory);
    std::shared_ptr<RobotBase> createMobileRobot(const std::shared_ptr<ModelFactory> factory);
    std::shared_ptr<RobotBase> createSerialRobot(const std::shared_ptr<ModelFactory> factory, RobotType type = RobotType::Serial);

    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<RobotBase> m_robot = nullptr;

    std::vector<std::string> m_obstaclePaths;
    AABB m_workspceBounding;

    FactoryType m_factoryType = FactoryType::ModelTriangle2D;
    RobotType m_robotType = RobotType::Point;

    // robot base properties
    size_t m_robotDim;
    std::vector<DofType> m_dofTypes;
    std::pair<VectorX, VectorX> m_robotBoundaries;
    std::string m_robotBaseModelFile;

    // serial robot properties
    std::vector<DhParameter> m_dhParameters;
    std::vector<std::string> m_linkModelFiles;
    Transform m_baseOffset;
    std::vector<Transform> m_linkOffsets;

    // obstacles
    std::vector<std::pair<std::string, Transform>> m_obstacleFilePaths;
};

} /* namespace ippp */

#endif    // ENVIRONMENTCONFIGURATOR_H
