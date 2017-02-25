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

#ifndef COLLISIONDETECTIONTRIANGLEROBOT_HPP
#define COLLISIONDETECTIONTRIANGLEROBOT_HPP

#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <robot/TriangleRobot2D.h>
#include <robot/model/Model2D.h>

namespace rmpl {

/*!
* \brief   Collision detection class for the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
class CollisionDetectionTriangleRobot : public CollisionDetection<3> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<RobotBase<3>> &robot);
    bool controlVec(const Vector3 &vec) override;
    bool controlTrajectory(std::vector<Vector3> &vec) override;

  private:
    bool checkPoint2D(float x, float y);
    bool checkTriangleRobot(const Vector3 &vec);

    Eigen::MatrixXi m_workspace2D;
    std::vector<Triangle2D> m_triangles;
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionTriangleRobot
*  \details    Creates local copy of base model and workspace of the robot
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
CollisionDetectionTriangleRobot::CollisionDetectionTriangleRobot(const std::shared_ptr<RobotBase<3>> &robot)
    : CollisionDetection("CollisionDetectionTriangleRobot", robot) {
    if (!m_robot->getWorkspace() || m_robot->getWorkspace()->empty()) {
        Logging::error("Empty workspace model", this);
        return;
    } else {
        m_workspace2D = std::dynamic_pointer_cast<Model2D>(robot->getWorkspace())->m_space;
    }
    if (!m_robot->getBaseModel() || m_robot->getBaseModel()->empty()) {
        Logging::error("Empty base model", this);
        return;
    } else {
        m_triangles = std::dynamic_pointer_cast<ModelTriangle2D>(m_robot->getBaseModel())->m_triangles;
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::controlVec(const Vector3 &vec) {
    return checkTriangleRobot(vec);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
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
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::checkPoint2D(float x, float y) {
    if (this->m_minBoundary[0] >= x || x >= this->m_maxBoundary[0] || this->m_minBoundary[1] >= y ||
        y >= this->m_maxBoundary[1]) {
        Logging::debug("Point out of workspace", this);
        return true;
    }

    if (m_workspace2D(x, y) < 80)
        return true;
    else
        return false;
}

/*!
*  \brief      Check for TriangleRobot collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision, true if in collision
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::checkTriangleRobot(const Vector3 &vec) {
    VectorX vector = vec;
    vector.resize(3);

    Matrix2 R;
    Vector2 t;
    utilGeo::poseVecToRandT(vector, R, t);

    Vector2 u, temp;
    std::vector<Triangle2D> triangles = m_triangles;
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

#endif /* COLLISIONDETECTIONTRIANGLEROBOT_HPP */
