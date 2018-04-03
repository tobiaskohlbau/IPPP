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

#include <ippp/environment/model/ModelTriangle2D.h>
#include <ippp/environment/robot/PointRobot.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Class for 2D collision detection of an point robot.
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection2D : public CollisionDetection<dim> {
  public:
    CollisionDetection2D(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());

    bool check(const Vector<dim> &config) const;
    bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  private:
    bool checkPoint2D(double x, double y) const;

    std::vector<Mesh> m_obstacles;

    using ValidityChecker<dim>::m_environment;
    using ValidityChecker<dim>::checkRobotBound;
};

/*!
*  \brief      Constructor of the class CollisionDetection2D
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection2D<dim>::CollisionDetection2D(const std::shared_ptr<Environment> &environment, const CollisionRequest &request)
    : CollisionDetection<dim>("CollisionDetection2D", environment, request) {
    assert(dim % 2 == 0);
    if (m_environment->numObstacles() == 0) {
        Logging::info("Empty workspace", this);
    } else {
        for (auto obstacle : m_environment->getObstacles())
            m_obstacles.push_back(obstacle->model->m_mesh);
        // update obstacle models for the 2D collision check, extends the AABB of the obstacle in z direction
        for (auto &obstacle : m_obstacles) {
            Vector3 bottomLeft = obstacle.aabb.min();
            Vector3 topRight = obstacle.aabb.max();
            bottomLeft[2] = -1;
            topRight[2] = 1;
            obstacle.aabb = AABB(bottomLeft, topRight);
        }
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
bool CollisionDetection2D<dim>::check(const Vector<dim> &config) const {
    if (!checkRobotBound(config))
        return false;
    return checkPoint2D(config[0], config[1]);
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
bool CollisionDetection2D<dim>::check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const {
    if (!checkRobotBound(config))
        return false;
    return checkPoint2D(config[0], config[1]);
}

/*!
*  \brief      Check collision of a trajectory of configurations
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetection2D<dim>::check(const std::vector<Vector<dim>> &configs) const {
    if (configs.empty() || !checkRobotBound(configs))
        return false;

    for (auto &config : configs)
        if (!checkPoint2D(config[0], config[1]))
            return false;

    return true;
}

/*!
*  \brief      Check for 2D point collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] binary result of collision (true if valid)
*  \date       2016-06-30
*/
template <unsigned int dim>
bool CollisionDetection2D<dim>::checkPoint2D(double x, double y) const {
    double alpha, beta, gamma;
    Vector3 p1, p2, p3;
    for (auto &obstacle : m_obstacles) {
        // check bounding box to point
        if (obstacle.aabb.exteriorDistance(Vector3(x, y, 0)) != 0)
            continue;

        // check if point is in triangle
        for (auto &face : obstacle.faces) {
            p1 = obstacle.vertices[face[0]];
            p2 = obstacle.vertices[face[1]];
            p3 = obstacle.vertices[face[2]];
            alpha = ((p2[1] - p3[1]) * (x - p3[0]) + (p3[0] - p2[0]) * (y - p3[1])) /
                    ((p2[1] - p3[1]) * (p1[0] - p3[0]) + (p3[0] - p2[0]) * (p1[1] - p3[1]));
            beta = ((p3[1] - p1[1]) * (x - p3[0]) + (p1[0] - p3[0]) * (y - p3[1])) /
                   ((p2[1] - p3[1]) * (p1[0] - p3[0]) + (p3[0] - p2[0]) * (p1[1] - p3[1]));
            gamma = 1.0f - alpha - beta;

            if (alpha > 0 && beta > 0 && gamma > 0)
                return false;
        }
    }
    return true;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION2D_HPP */
