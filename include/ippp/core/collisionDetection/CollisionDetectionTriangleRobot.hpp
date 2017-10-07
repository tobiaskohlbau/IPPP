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

#ifndef COLLISIONDETECTIONTRIANGLEROBOT_HPP
#define COLLISIONDETECTIONTRIANGLEROBOT_HPP

#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/environment/cad/CadDrawing.h>
#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/environment/robot/TriangleRobot2D.h>

namespace ippp {

/*!
* \brief   Collision detection class for the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionTriangleRobot : public CollisionDetection<dim> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment);
    bool checkConfig(const Vector<dim> &config, CollisionData *data = nullptr) override;
    bool checkTrajectory(std::vector<Vector<dim>> &configs) override;

  private:
    bool checkPoint2D(double x, double y);
    bool checkTriangleRobot(const Vector<dim> &vec);
    bool lineTriangle(const Vector3 p, const Vector3 q, const Vector3 a, const Vector3 b, const Vector3 c);

    Eigen::MatrixXi m_workspace2D;
    std::vector<Triangle2D> m_triangles;
    std::shared_ptr<ModelContainer> m_robotModel;
    AABB m_workspaceBounding;
    std::pair<Vector3, Vector3> m_robotBounding;
    std::vector<Mesh> m_obstacles;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionTriangleRobot
*  \details    Creates local copy of base model and workspace of the robot
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionTriangleRobot<dim>::CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment)
    : CollisionDetection<dim>("CollisionDetectionTriangleRobot", environment) {
    assert(dim % 3 == 0);

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
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::checkConfig(const Vector<dim> &config, CollisionData *data) {
    return checkTriangleRobot(config);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::checkTrajectory(std::vector<Vector<dim>> &configs) {
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
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::checkPoint2D(double x, double y) {
    if (m_workspaceBounding.min()[0] >= x || x >= m_workspaceBounding.max()[0] || m_workspaceBounding.min()[1] >= y ||
        y >= m_workspaceBounding.max()[1]) {
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
            s = 1 / (2 * area) * (p0[1] * p2[0] - p0[0] * p2[1] + (p2[1] - p0[1]) * x + (p0[0] - p2[0]) * y);
            t = 1 / (2 * area) * (p0[0] * p1[1] - p0[1] * p1[0] + (p0[1] - p1[1]) * x + (p1[0] - p0[0]) * y);

            if (s > 0 && t > 0 && 1 - s - t > 0)
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
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::checkTriangleRobot(const Vector<dim> &config) {
    // check boundary of the robot
    for (unsigned int i = 0; i < 3; ++i) {
        if (config[i] < m_robotBounding.first[i] || m_robotBounding.second[i] < config[i]) {
            Logging::trace("Out of robot boundary", this);
            return true;
        }
    }

    if (m_obstacles.empty())
        return false;

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
    Matrix2 R = trafo.first.block(0, 0, 2, 2);

    Vector2 u, temp;
    std::vector<Triangle2D> triangles = m_triangles;
    for (auto triangle : triangles) {
        triangle.transform(R, t);
        Vector3 p1(triangle.getP(1)[0], triangle.getP(1)[1], 0);
        Vector3 p2(triangle.getP(2)[0], triangle.getP(2)[1], 0);
        Vector3 p3(triangle.getP(3)[0], triangle.getP(3)[1], 0);

        for (auto &ob : m_obstacles) {
            // line of robot triangle and triangle from obstacle
            for (auto &face : ob.faces) {
                if (lineTriangle(p1, p2, ob.vertices[face[0]], ob.vertices[face[1]], ob.vertices[face[2]]))
                    return true;
                if (lineTriangle(p3, p1, ob.vertices[face[0]], ob.vertices[face[1]], ob.vertices[face[2]]))
                    return true;
                if (lineTriangle(p3, p2, ob.vertices[face[0]], ob.vertices[face[1]], ob.vertices[face[2]]))
                    return true;
            }
        }

        //        u = triangle.getP(2) - triangle.getP(1);
        //        u /= u.norm();
        //        for (temp = triangle.getP(1) + u; (temp - triangle.getP(2)).squaredNorm() > 2; temp += u) {
        //            if (checkPoint2D(temp[0], temp[1])) {
        //                return true;
        //            }
        //        }
        //        u = triangle.getP(3) - triangle.getP(1);
        //        u /= u.norm();
        //        for (temp = triangle.getP(1) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u) {
        //            if (checkPoint2D(temp[0], temp[1])) {
        //                return true;
        //            }
        //        }
        //        u = triangle.getP(3) - triangle.getP(2);
        //        u /= u.norm();
        //        for (temp = triangle.getP(2) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u) {
        //            if (checkPoint2D(temp[0], temp[1])) {
        //                return true;
        //            }
        //        }
    }
    return false;
}

template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::lineTriangle(const Vector3 p, const Vector3 q, const Vector3 a, const Vector3 b,
                                                        const Vector3 c) {
    Vector3 pq = q - p;
    Vector3 pa = a - p;
    Vector3 pb = b - p;
    Vector3 pc = c - p;
    // Test if pq is inside the edges bc, ca and ab. Done by testing that the signed tetrahedral volumes, computed using scalar
    // triple products, are all positive
    double u = pq.dot(pc.cross(pb));
    if (u < 0.0)
        return false;
    double v = pq.dot(pa.cross(pc));
    if (v < 0.0)
        return false;
    double w = pq.dot(pb.cross(pa));
    if (w < 0.0)
        return false;

    return true;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONTRIANGLEROBOT_HPP */
