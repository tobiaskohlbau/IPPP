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

#ifndef COLLISIONDETECTIONTRIANGLEROBOT_H
#define COLLISIONDETECTIONTRIANGLEROBOT_H

#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/environment/CadProcessing.h>
#include <ippp/environment/robot/TriangleRobot2D.h>

namespace ippp {

/*!
* \brief   Collision detection class for the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
class CollisionDetectionTriangleRobot : public CollisionDetection<3> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment);
    bool checkConfig(const Vector3 &config, CollisionData *data = nullptr) override;
    bool checkTrajectory(std::vector<Vector3> &configs) override;

  private:
    bool checkPoint2D(double x, double y);
    bool checkTriangleRobot(const Vector3 &vec);

    Eigen::MatrixXi m_workspace2D;
    std::vector<Triangle2D> m_triangles;
    std::shared_ptr<ModelContainer> m_robotModel;
    AABB m_workspaceBounding;
    std::pair<Vector3, Vector3> m_robotBounding;
    std::vector<Mesh> m_obstacles;
};

} /* namespace ippp */

#endif /* COLLISIONDETECTIONTRIANGLEROBOT_H */
