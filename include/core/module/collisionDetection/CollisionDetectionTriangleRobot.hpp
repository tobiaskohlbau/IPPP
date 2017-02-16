//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef COLLISIONDETECTIONTRIANGLEROBOT_H_
#define COLLISIONDETECTIONTRIANGLEROBOT_H_

#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <robot/TriangleRobot2D.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return binary value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetectionTriangleRobot : public CollisionDetection<3> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<RobotBase<3>> &robot);
    bool controlVec(const Vector3 &vec) override;
    bool controlTrajectory(std::vector<Vector3> &vec) override;

  private:
    bool checkPoint2D(float x, float y);
    bool checkTriangleRobot(const Vector3 &vec);
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
CollisionDetectionTriangleRobot::CollisionDetectionTriangleRobot(const std::shared_ptr<RobotBase<3>> &robot) : CollisionDetection("CollisionDetectionTriangleRobot", robot) {
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vec
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
bool CollisionDetectionTriangleRobot::controlVec(const Vector3 &vec) {
    return checkTriangleRobot(vec);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of points
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
bool CollisionDetectionTriangleRobot::controlTrajectory(std::vector<Vector3> &vecs) {
    if (vecs.size() == 0)
        return false;

    for (int i = 0; i < vecs.size(); ++i)
        if (checkTriangleRobot(vecs[i]))
            return true;

    return false;
}

/*!
*  \brief      Check for 2D point collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-06-30
*/
bool CollisionDetectionTriangleRobot::checkPoint2D(float x, float y) {
    if (this->m_minBoundary[0] >= x || x >= this->m_maxBoundary[0] || this->m_minBoundary[1] >= y || y >= this->m_maxBoundary[1]) {
        Logging::debug("Point out of workspace", this);
        return true;
    }

    if (m_2DWorkspace(x, y) < 80)
        return true;
    else
        return false;
}

/*!
*  \brief      Check for TriangleRobot collision
*  \author     Sascha Kaden
*  \param[in]  transformation Vector
*  \param[out] binary result of collision, true if in collision
*  \date       2016-12-19
*/
bool CollisionDetectionTriangleRobot::checkTriangleRobot(const Vector3 &vec) {
    std::shared_ptr<TriangleRobot2D> robot(std::dynamic_pointer_cast<TriangleRobot2D>(m_robot));
    std::vector<Triangle2D> triangles = robot->getTriangles();
    VectorX vector = vec;
    vector.resize(3);

    Matrix2 R;
    Vector2 t;
    utilGeo::poseVecToRandT(vector, R, t);

    Vector2 u, temp;
    for (auto triangle : triangles) {
        triangle.transform(R, t);

        u = triangle.getP(2) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(2)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;

        u = triangle.getP(3) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;

        u = triangle.getP(3) - triangle.getP(2);
        u /= u.norm();
        for (temp = triangle.getP(2) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;
    }

    return false;
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTIONTRIANGLEROBOT_H_ */
