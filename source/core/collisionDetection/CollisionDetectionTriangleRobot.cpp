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

#include <ippp/core/collisionDetection/CollisionDetectionTriangleRobot.h>

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
    auto robot = m_environment->getRobot();

    // set boundaries
    m_workspaceBounding = m_environment->getBoundary();
    m_robotBounding = std::make_pair(robot->getMinBoundary(), robot->getMaxBoundary());

    // create workspace from the 2d triangles
    m_workspace2D = cad::create2dspace(m_environment->getBoundary(), 255);

    if (m_environment->getObstacleNum() == 0) {
        Logging::warning("Empty workspace", this);
    } else {
        for (auto obstacle : m_environment->getObstacles())
            m_obstacles.push_back(obstacle->m_mesh);
    }

    // update obstacle models for the 2D collision check, extends the AABB of the obstacle
    for (auto obstacle : m_obstacles) {
        Vector3 bottomLeft = obstacle.aabb.corner(AABB::CornerType::BottomLeft);
        Vector3 topRight = obstacle.aabb.corner(AABB::CornerType::TopRight);
        bottomLeft[2] = -1;
        topRight[2] = 1;
        obstacle.aabb = AABB(bottomLeft, topRight);
    }


    if (!robot->getBaseModel() || robot->getBaseModel()->empty()) {
        Logging::error("Empty base model", this);
        return;
    } else {
        m_robotModel = robot->getBaseModel();
        m_triangles = std::dynamic_pointer_cast<ModelTriangle2D>(m_robotModel)->m_triangles;
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::checkConfig(const Vector3 &config, CollisionData *data) {
    return checkTriangleRobot(config);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::checkTrajectory(std::vector<Vector3> &configs) {
    if (configs.empty())
        return false;

    for (auto &config : configs)
        if (checkTriangleRobot(config))
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
bool CollisionDetectionTriangleRobot::checkPoint2D(double x, double y) {
    if (m_workspaceBounding.min()[0] >= x || x >= m_workspaceBounding.max()[0] ||
        m_workspaceBounding.min()[1] >= y || y >= m_workspaceBounding.max()[1]) {
        Logging::trace("Config out of workspace bound", this);
        return true;
    }

    double s, t, area;
    Vector3 p0, p1, p2;
    for (auto &obstacle : m_obstacles) {
        // check if point is in triangle
        for (auto &face : obstacle.faces) {
            p0 = obstacle.vertices[face[0]];
            p1 = obstacle.vertices[face[1]];
            p2 = obstacle.vertices[face[2]];
            area = std::abs(0.5 * (-p1[1] * p2[0] + p0[1] * (-p1[0] + p2[0]) + p0[0] * (p1[1] - p2[1]) + p1[0] * p2[1]));
            s = 1/(2*area)*(p0[1]*p2[0] - p0[0]*p2[1] + (p2[1] - p0[1])*x + (p0[0] - p2[0])*y);
            t = 1/(2*area)*(p0[0]*p1[1] - p0[1]*p1[0] + (p0[1] - p1[1])*x + (p1[0] - p0[0])*y);

            if (s>0 && t>0 && 1-s-t>0)
                return true;
        }
    }
    return false;
}

/*!
*  \brief      Check for TriangleRobot collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision, true if in collision
*  \date       2017-02-19
*/
bool CollisionDetectionTriangleRobot::checkTriangleRobot(const Vector3 &config) {
    // check boundary of the robot
    for (unsigned int i = 0; i < 3; ++i) {
        if (config[i] < m_robotBounding.first[i] || m_robotBounding.second[i] < config[i]) {
            Logging::trace("Out of robot boundary", this);
            return true;
        }
    }

    auto trafo = m_environment->getRobot()->getTransformation(config);
    // bounding box check
    AABB robotAABB = util::transformAABB(m_robotModel->m_mesh.aabb, trafo);
    bool intersection = false;
    for (auto &obstacle : m_obstacles) {
        if (robotAABB.intersects(obstacle.aabb)) {
            intersection = true;
            break;
        }
    }
    if (!intersection)
        return false;

    Vector2 t(trafo.second[0], trafo.second[1]);
    Matrix2 R = trafo.first.block<2,2>(0,0);

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
