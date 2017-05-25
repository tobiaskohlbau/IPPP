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

#include <core/collisionDetection/CollisionDetectionTriangleRobot.h>

namespace ippp {

/*!
*  \brief      Standard constructor of the class CollisionDetectionTriangleRobot
*  \details    Creates local copy of base model and workspace of the robot
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
CollisionDetectionTriangleRobot::CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment)
    : CollisionDetection("CollisionDetectionTriangleRobot", environment) {
    // set boundaries
    auto bound = m_environment->getBoundary();
    m_minBoundary = Vector2(bound.min()[0], bound.min()[1]);
    m_maxBoundary = Vector2(bound.max()[0], bound.max()[1]);

    // create workspace from the 2d triangles
    m_workspace2D = cad::create2dspace(m_environment->getBoundary(), 255);

    if (!m_environment->getObstacleNum() == 0) {
        Logging::warning("Empty workspace", this);
    } else {
        std::vector<std::shared_ptr<ModelContainer>> obstacles = m_environment->getObstacles();
        for (auto &obstacle : obstacles) {
            auto model = std::dynamic_pointer_cast<ModelTriangle2D>(obstacle);
            cad::drawTriangles(m_workspace2D, model->m_triangles, 0);
        }
    }

    auto robot = m_environment->getRobot();
    if (!robot->getBaseModel() || robot->getBaseModel()->empty()) {
        Logging::error("Empty base model", this);
        return;
    } else {
        m_triangles = std::dynamic_pointer_cast<ModelTriangle2D>(robot->getBaseModel())->m_triangles;
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
    if (vecs.size() == 0) {
        return false;
    }
    for (int i = 0; i < vecs.size(); ++i) {
        if (checkTriangleRobot(vecs[i])) {
            return true;
        }
    }
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
bool CollisionDetectionTriangleRobot::checkPoint2D(double x, double y) {
    if (m_minBoundary[0] >= x || x >= m_maxBoundary[0] || m_minBoundary[1] >= y || y >= m_maxBoundary[1]) {
        Logging::debug("Point out of workspace", this);
        return true;
    }

    if (m_workspace2D(y, x) < 80) {
        return true;
    } else {
        return false;
    }
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
    util::poseVecToRandT(vector, R, t);

    Vector2 u, temp;
    std::vector<Triangle2D> triangles = m_triangles;
    for (auto triangle : triangles) {
        triangle.transform(R, t);

        u = triangle.getP(2) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(2)).squaredNorm() > 2; temp += u) {
            if (checkPoint2D(temp[0], temp[1])) {
                return true;
            }
        }
        u = triangle.getP(3) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u) {
            if (checkPoint2D(temp[0], temp[1])) {
                return true;
            }
        }
        u = triangle.getP(3) - triangle.getP(2);
        u /= u.norm();
        for (temp = triangle.getP(2) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u) {
            if (checkPoint2D(temp[0], temp[1])) {
                return true;
            }
        }
    }
    return false;
}

} /* namespace ippp */
