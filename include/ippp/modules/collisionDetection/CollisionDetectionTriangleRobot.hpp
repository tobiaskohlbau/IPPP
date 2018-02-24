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

#include <ippp/environment/cad/CadDrawing.h>
#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/environment/robot/TriangleRobot2D.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Collision detection class for the TriangleRobot2D
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionTriangleRobot : public CollisionDetection<dim> {
  public:
    CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment,
                                    const CollisionRequest &request = CollisionRequest());
    bool check(const Vector<dim> &config) const;
    bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  private:
    bool checkTriangles(const Transform &T, const std::vector<Triangle2D> &triangles) const;
    bool lineTriangle(const Vector3 p, const Vector3 q, const Vector3 a, const Vector3 b, const Vector3 c) const;

    std::shared_ptr<ModelContainer> m_robotModel;
    std::vector<Triangle2D> m_baseTriangles;

    AABB m_workspaceBounding;
    std::vector<Mesh> m_obstacles;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionTriangleRobot
*  \details    Creates local copy of base model and workspace of the robot
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionTriangleRobot<dim>::CollisionDetectionTriangleRobot(const std::shared_ptr<Environment> &environment,
                                                                      const CollisionRequest &request)
    : CollisionDetection<dim>("CollisionDetectionTriangleRobot", environment, request) {
    assert(dim % 3 == 0);

    auto robot = m_environment->getRobot();

    // set boundaries
    m_workspaceBounding = m_environment->getSpaceBoundary();
    if (m_environment->numObstacles() == 0) {
        Logging::warning("Empty workspace", this);
    } else {
        for (auto obstacle : m_environment->getObstacles())
            m_obstacles.push_back(obstacle->model->m_mesh);
    }

    // update obstacle models for the 2D collision check, extends the AABB of the obstacle
    for (auto &obstacle : m_obstacles) {
        for (auto &vertex : obstacle.vertices)
            vertex[2] = 0;
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
        m_baseTriangles = std::dynamic_pointer_cast<ModelTriangle2D>(robot->getBaseModel())->m_triangles;
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::check(const Vector<dim> &config) const {
    if (m_obstacles.empty())
        return true;

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
        return true;

    return checkTriangles(trafo, m_baseTriangles);
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[in]  CollisionRequest
*  \param[out] CollisionResult
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::check(const Vector<dim> &config, const CollisionRequest &request,
                                                 CollisionResult &result) const {
    return check(config);
}

/*!
*  \brief      Check collision of a trajectory of configurations
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::check(const std::vector<Vector<dim>> &configs) const {
    for (auto &config : configs)
        if (check(config))
            return false;

    return true;
}

template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::checkTriangles(const Transform &T, const std::vector<Triangle2D> &triangles) const {
    std::vector<Triangle2D> tempTriangles = triangles;
    Matrix2 R2D = T.rotation().block<2, 2>(0, 0);
    Vector2 t2D = T.translation().block<2, 1>(0, 0);
    for (auto triangle : tempTriangles) {
        triangle.transform(R2D, t2D);
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
    }
    return false;
}

template <unsigned int dim>
bool CollisionDetectionTriangleRobot<dim>::lineTriangle(const Vector3 p, const Vector3 q, const Vector3 a, const Vector3 b,
                                                        const Vector3 c) const {
    Vector3 pq = q - p;
    Vector3 pa = a - p;
    Vector3 pb = b - p;
    Vector3 pc = c - p;
    // Test if pq is inside the edges bc, ca and ab. Done by testing that the signed tetrahedral volumes, computed using
    // scalar
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
