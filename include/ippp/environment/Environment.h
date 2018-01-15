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

#include <assert.h>
#include <string>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/environment/ObstacleObject.h>
#include <ippp/environment/model/ModelContainer.h>
#include <ippp/environment/robot/RobotBase.h>

namespace ippp {

enum class BodyType { Planar, Volumetric };

/*!
* \brief   Environment class, contains a list of obstacles, the workspace boundaries and the robot.
* \author  Sascha Kaden
* \date    2017-05-16
*/
class Environment : public Identifier {
  public:
    Environment(const AABB &spaceBoundary);
    Environment(const AABB &spaceBoundary, const std::shared_ptr<EnvObject> &robot);
    Environment(const AABB &spaceBoundary, const std::vector<std::shared_ptr<EnvObject>> &robots);
    ~Environment();

    void addEnvObject(const std::shared_ptr<EnvObject> &object);
    void addEnvObjects(const std::vector<std::shared_ptr<EnvObject>> &objects);
    std::vector<std::shared_ptr<EnvObject>> getObjects() const;
    std::shared_ptr<EnvObject> getObject(size_t index) const;
    size_t numObjects() const;

    std::vector<std::shared_ptr<ObstacleObject>> getObstacles() const;
    size_t numObstacles() const;
    std::vector<std::shared_ptr<RobotBase>> getRobots() const;
    std::shared_ptr<RobotBase> getRobot() const;
    size_t numRobots() const;

    AABB getSpaceBoundary() const;
    std::pair<VectorX, VectorX> getRobotBoundaries() const;
    std::vector<unsigned int> getRobotDimSizes() const;
    unsigned int getConfigDim() const;
    std::pair<VectorX, VectorX> getConfigMasks() const;

  protected:
    void update();
    
    void updateConfigDim();
    void updateMasks();
    void updateRobotBoundaries();

    unsigned int m_configDim = 0; /*!< summarized configuration dimension of all robots */
    const AABB m_spaceBoundary;   /*!< 3D boundary of the euclidean workspace */
    std::vector<unsigned int> m_robotDimSizes;                /*!< list of dimension size of each robot */
    std::pair<VectorX, VectorX> m_robotBoundaries;

    std::vector<std::shared_ptr<EnvObject>> m_envObjects; /*!< list with pointers to the environment objects */
    VectorX m_positionMask;
    VectorX m_rotationMask;
};

} /* namespace ippp */

#endif /* ENVIRONMENT_H */
