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

#ifndef COLLISIONDETECTIONSPHERE_HPP
#define COLLISIONDETECTIONSPHERE_HPP

#include <Eigen/Geometry>

#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>

namespace ippp {

/*!
* \brief   Class for AABB collision detection with the Eigen AABB structure, which is saved in the Mesh container.
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionSphere : public CollisionDetection<dim> {
  public:
    CollisionDetectionSphere(const std::shared_ptr<Environment> &environment,
                             const CollisionRequest &request = CollisionRequest());
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr) override;
    bool checkTrajectory(std::vector<Vector<dim>> &configs) override;

  private:
    bool checkObstacles(const AABB &robotAABB, CollisionResult *result);
    bool checkRobots(const std::vector<AABB> &robots, CollisionResult *result);
    double checkSphere(const AABB &a, const AABB &b);

    bool m_multiRobot = false;
    std::vector<AABB> m_robotAABBs;
    std::vector<AABB> m_obstacleAABBs;
    std::vector<std::shared_ptr<RobotBase>> m_robots;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Constructor of the class CollisionDetectionAABB
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionSphere<dim>::CollisionDetectionSphere(const std::shared_ptr<Environment> &environment,
                                                        const CollisionRequest &request)
    : CollisionDetection<dim>("CollisionDetectionSphere", environment, request) {
    if (environment->numRobots() > 1)
        m_multiRobot = true;

    m_robots = m_environment->getRobots();

    for (const auto &robot : environment->getRobots())
        m_robotAABBs.push_back(robot->getBaseModel()->getAABB());

    for (const auto &obstacle : environment->getObstacles())
        m_obstacleAABBs.push_back(obstacle->getAABB());
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision or config is empty)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetectionSphere<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *request, CollisionResult *result) {
    CollisionRequest collisionRequest;
    if (request != nullptr)
        collisionRequest = *request;
    else
        collisionRequest = this->m_request;

    if (m_multiRobot && collisionRequest.checkInterRobot) {
        // compute the new AABBs of the robots with the configuration
        std::vector<VectorX> singleConfigs = util::splitVec<dim>(config, m_environment->getRobotDimSizes());
        std::vector<AABB> robotAABBs;
        for (unsigned int i = 0; i < m_robots.size(); ++i) {
            auto trafo = m_robots[i]->getTransformation(singleConfigs[i]);
            robotAABBs.push_back(util::translateAABB(m_robotAABBs[i], trafo));
        }
        // check collisions
        if (checkRobots(robotAABBs, result))
            return true;
        for (auto &robotAABB : robotAABBs)
            if (checkObstacles(robotAABB, result))
                return true;
    } 
    if (collisionRequest.checkObstacle) {
        auto trafo = m_robots[0]->getTransformation(config);
        AABB robotAABB = util::translateAABB(m_robotAABBs[0], trafo);
        return checkObstacles(robotAABB, result);
    }

    return false;
}

/*!
*  \brief      Check collision of a trajectory of configurations
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetectionSphere<dim>::checkTrajectory(std::vector<Vector<dim>> &configs) {
    if (configs.empty())
        return false;

    for (auto &config : configs)
        if (checkConfig(config))
            return true;

    return false;
}

template <unsigned int dim>
bool CollisionDetectionSphere<dim>::checkRobots(const std::vector<AABB> &robots, CollisionResult *result) {
    if (result != nullptr) {
        double dist;
        for (auto a = robots.begin(); a != robots.end() - 1; ++a) {
            for (auto b = robots.begin() + 1; b != robots.end(); ++b) {
                dist = checkSphere(*a, *b);
                if (dist < result->minDist)
                    result->minDist = dist;
                if (dist < result->minRobotDist)
                    result->minRobotDist = dist;
                if (dist == 0) {
                    result->collision = true;
                    return true;
                }
            }
        }
    } else {
        for (auto a = robots.begin(); a != robots.end() - 1; ++a)
            for (auto b = robots.begin() + 1; b != robots.end(); ++b)
                if (checkSphere(*a, *b) < 0)
                    return true;
    }

    return false;
}

template <unsigned int dim>
bool CollisionDetectionSphere<dim>::checkObstacles(const AABB &robotAABB, CollisionResult *result) {
    if (result != nullptr) {
        double dist;
        for (auto &obstacle : m_obstacleAABBs) {
            dist = checkSphere(robotAABB, obstacle);
            if (dist < result->minDist)
                result->minDist = dist;
            if (dist < result->minObstacleDist)
                result->minObstacleDist = dist;
            if (dist == 0) {
                result->collision = true;
                return true;
            }
        }
    } else {
        for (auto &obstacle : m_obstacleAABBs)
            if (checkSphere(robotAABB, obstacle) < 0)
                return true;
    }

    return false;
}

template <unsigned int dim>
double CollisionDetectionSphere<dim>::checkSphere(const AABB &a, const AABB &b) {
    // Book: Real-Time Collision Detection page 88
    Vector3 d = a.center() - b.center();
    double dist2 = d.dot(d);
    // Spheres intersect if squared distance is less than squared sum of radii
    double radiusSum = a.diagonal().norm() + b.diagonal().norm();
    return dist2 - (radiusSum * radiusSum);
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONAABB_HPP */
