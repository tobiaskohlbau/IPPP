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

#ifndef COLLISIONDETECTION2D_HPP
#define COLLISIONDETECTION2D_HPP

#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/environment/CadProcessing.h>
#include <ippp/environment/model/ModelTriangle2D.h>
#include <ippp/environment/robot/PointRobot.h>

namespace ippp {

/*!
* \brief   Class for 2D collision detection of an point robot.
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection2D : public CollisionDetection<dim> {
  public:
    CollisionDetection2D(const std::shared_ptr<Environment> &environment);
    bool checkConfig(const Vector2 &config, CollisionData *data = nullptr) override;
    bool checkTrajectory(std::vector<Vector2> &configs) override;

  private:
    bool checkPoint2D(double x, double y);

    Vector2 m_minBoundary;
    Vector2 m_maxBoundary;
    std::vector<Mesh> m_obstacles;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Constructor of the class CollisionDetection2D
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection2D<dim>::CollisionDetection2D(const std::shared_ptr<Environment> &environment)
    : CollisionDetection<dim>("CollisionDetection2D", environment) {
    assert(dim % 2 == 0);
    // set boundaries
    auto bound = m_environment->getBoundary();
    m_minBoundary = Vector2(bound.min()[0], bound.min()[1]);
    m_maxBoundary = Vector2(bound.max()[0], bound.max()[1]);

    if (m_environment->getObstacleNum() == 0) {
        Logging::warning("Empty workspace", this);
    } else {
        for (auto obstacle : m_environment->getObstacles())
            m_obstacles.push_back(obstacle->m_mesh);
    }

    // update obstacle models for the 2D collision check, extends the AABB of the obstacle in z direction
    for (auto obstacle : m_obstacles) {
        Vector3 bottomLeft = obstacle.aabb.corner(AABB::CornerType::BottomLeft);
        Vector3 topRight = obstacle.aabb.corner(AABB::CornerType::TopRight);
        bottomLeft[2] = -1;
        topRight[2] = 1;
        obstacle.aabb = AABB(bottomLeft, topRight);
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision or vec is empty)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetection2D<dim>::checkConfig(const Vector2 &config, CollisionData *data) {
    return checkPoint2D(config[0], config[1]);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetection2D<dim>::checkTrajectory(std::vector<Vector2> &configs) {
    if (configs.empty())
        return false;

    for (auto &config : configs)
        if (checkPoint2D(config[0], config[1]))
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
template <unsigned int dim>
bool CollisionDetection2D<dim>::checkPoint2D(double x, double y) {
    if (m_minBoundary[0] >= x || x >= m_maxBoundary[0] || m_minBoundary[1] >= y || y >= m_maxBoundary[1]) {
        Logging::trace("Config out of bound", this);
        return true;
    }

    double s, t, area;
    Vector3 p0, p1, p2;
    for (auto &obstacle : m_obstacles) {
        // check bounding box to point
        if (obstacle.aabb.exteriorDistance(Vector3(x, y, 0)) != 0)
            continue;

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

} /* namespace ippp */

#endif /* COLLISIONDETECTION2D_HPP */
