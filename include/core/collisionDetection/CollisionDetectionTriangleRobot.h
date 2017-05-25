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

#include <core/collisionDetection/CollisionDetection.hpp>
#include <environment/CadProcessing.h>
#include <environment/robot/TriangleRobot2D.h>

namespace ippp {

/*!
* \brief   Collision detection class for the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
class CollisionDetectionTriangleRobot : public CollisionDetection<3> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment);
    bool controlVec(const Vector3 &vec) override;
    bool controlTrajectory(std::vector<Vector3> &vec) override;

  private:
    bool checkPoint2D(double x, double y);
    bool checkTriangleRobot(const Vector3 &vec);

    Eigen::MatrixXi m_workspace2D;
    std::vector<Triangle2D> m_triangles;
    Vector2 m_minBoundary;
    Vector2 m_maxBoundary;
};

} /* namespace ippp */

#endif /* COLLISIONDETECTIONTRIANGLEROBOT_H */
