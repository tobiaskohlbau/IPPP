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

#include <ui/Configurator.h>

#include <ippp/Environment.h>
#include <ippp/core/types.h>

namespace ippp {

enum class FactoryType { ModelFCL, ModelPQP, ModelTriangle2D };

enum class RobotType { Jaco, Kuka, Point, Triangle2D, Serial, Mobile };

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

    void setWorkspaceProperties(const unsigned int workspaceDim, const AABB &workspaceBounding);
    void setObstaclePaths(const std::vector<std::string> &obstaclePaths);
    void addObstaclePath(const std::string &obstaclePath);
    void setFactoryType(const FactoryType factoryType);
    void setRobotType(const RobotType robotType, const std::string &robotFile = "");

    std::shared_ptr<Environment> getEnvironment();

  protected:
    std::shared_ptr<Environment> m_environment = nullptr;
    std::shared_ptr<RobotBase> m_robot = nullptr;

    std::vector<std::string> m_obstaclePaths;
    unsigned int m_workspaceDim = 3;
    AABB m_workspceBounding;
    FactoryType m_factoryType = FactoryType::ModelTriangle2D;
    RobotType m_robotType = RobotType::Point;
    std::string m_robotFile;
};

} /* namespace ippp */

#endif    // ENVIRONMENTCONFIGURATOR_H
