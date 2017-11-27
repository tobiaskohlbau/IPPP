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

#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <cassert>
#include <string>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/environment/robot/RobotBase.h>
#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

enum class BodyType { Planar, Volumetric };

/*!
* \brief   Environment class, contains a list of obstacles, the workspace boundaries and the robot.
* \author  Sascha Kaden
* \date    2017-05-16
*/
class Environment : public Identifier {
  public:
    Environment(unsigned int workspaceDim, const AABB &spaceBoundary);
    Environment(unsigned int workspaceDim, const AABB &spaceBoundary, const std::shared_ptr<RobotBase> &robot);
    Environment(unsigned int workspaceDim, const AABB &spaceBoundary, const std::vector<std::shared_ptr<RobotBase>> &robots);
    ~Environment() override;

    void addObstacle(const std::shared_ptr<ModelContainer> &model);
    void addObstacles(const std::vector<std::shared_ptr<ModelContainer>> &models);
    std::shared_ptr<ModelContainer> getObstacle(size_t index) const;
    std::vector<std::shared_ptr<ModelContainer>> getObstacles() const;
    size_t getObstacleNum() const;

    void addRobot(const std::shared_ptr<RobotBase> &robot);
    std::shared_ptr<RobotBase> getRobot() const;
    std::shared_ptr<RobotBase> getRobot(size_t index) const;
    std::vector<std::shared_ptr<RobotBase>> getRobots() const;
    size_t numRobots() const;

    AABB getSpaceBoundary() const;
    unsigned int getSpaceDim() const;
    std::pair<VectorX, VectorX> getRobotBoundaries() const;
    std::vector<unsigned int> getRobotDimSizes() const;
    unsigned int getConfigDim() const;
    std::pair<VectorX, VectorX> getConfigMasks() const;

  protected:
    void updateConfigurationDim();
    void updateMasks();

    unsigned int m_configDim = 0;
    const unsigned int m_spaceDim = 0;
    const AABB m_spaceBoundary;

    std::vector<unsigned int> m_robotDimSizes;
    std::vector<std::shared_ptr<RobotBase>> m_robots;
    std::vector<std::shared_ptr<ModelContainer>> m_obstacles;
    VectorX m_positionMask;
    VectorX m_rotationMask;
};

} /* namespace ippp */

#endif /* ENVIRONMENT_H */
