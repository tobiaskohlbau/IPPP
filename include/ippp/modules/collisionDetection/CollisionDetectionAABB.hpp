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

#ifndef COLLISIONDETECTIONAABB_HPP
#define COLLISIONDETECTIONAABB_HPP

#include <Eigen/Geometry>

#include <ippp/environment/cad/CadProcessing.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
* \brief   Class for AABB collision detection with the Eigen AABB structure, which is saved in the Mesh container.
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionAABB : public CollisionDetection<dim> {
  public:
    CollisionDetectionAABB(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());

    bool check(const Vector<dim> &config) const;
    bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  private:
    bool checkObstacles(const AABB &robotAABB, CollisionResult *result = nullptr) const;
    bool checkRobots(const std::vector<AABB> &robotAABB, CollisionResult *result = nullptr) const;

    bool m_multiRobot = false;
    std::vector<AABB> m_robotAABBs;
    std::vector<AABB> m_obstacleAABBs;
    std::vector<std::shared_ptr<RobotBase>> m_robots;

    using CollisionDetection<dim>::m_environment;
    using CollisionDetection<dim>::m_request;
};

/*!
*  \brief      Constructor of the class CollisionDetectionAABB
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionAABB<dim>::CollisionDetectionAABB(const std::shared_ptr<Environment> &environment,
                                                    const CollisionRequest &request)
    : CollisionDetection<dim>("CollisionDetectionAABB", environment, request) {
    if (environment->numRobots() > 1)
        m_multiRobot = true;

    m_robots = m_environment->getRobots();

    for (auto &robot : environment->getRobots())
        m_robotAABBs.push_back(robot->getBaseModel()->getAABB());

    for (auto &obstacle : environment->getObstacles())
        m_obstacleAABBs.push_back(obstacle->model->getAABB());
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetectionAABB<dim>::check(const Vector<dim> &config) const {
    if (m_multiRobot && m_request.checkInterRobot) {
        // compute the new AABBs of the robots with the configuration
        std::vector<VectorX> singleConfigs = util::splitVec<dim>(config, m_environment->getRobotDimSizes());
        std::vector<AABB> robotAABBs;
        for (unsigned int i = 0; i < m_robots.size(); ++i) {
            auto trafo = m_robots[i]->getTransformation(singleConfigs[i]);
            robotAABBs.push_back(util::transformAABB(m_robotAABBs[i], trafo));
        }
        // check collisions
        if (checkRobots(robotAABBs))
            return false;
        for (auto &robotAABB : robotAABBs)
            if (checkObstacles(robotAABB))
                return false;
    }
    if (m_request.checkObstacle) {
        auto trafo = m_robots[0]->getTransformation(config);
        AABB robotAABB = util::transformAABB(m_robotAABBs[0], trafo);
        return !checkObstacles(robotAABB);
    }

    return true;
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
bool CollisionDetectionAABB<dim>::check(const Vector<dim> &config, const CollisionRequest &request,
                                        CollisionResult &result) const {
    if (m_multiRobot && request.checkInterRobot) {
        // compute the new AABBs of the robots with the configuration
        std::vector<VectorX> singleConfigs = util::splitVec<dim>(config, m_environment->getRobotDimSizes());
        std::vector<AABB> robotAABBs;
        for (unsigned int i = 0; i < m_robots.size(); ++i) {
            auto trafo = m_robots[i]->getTransformation(singleConfigs[i]);
            robotAABBs.push_back(util::transformAABB(m_robotAABBs[i], trafo));
        }
        // check collisions
        if (checkRobots(robotAABBs, &result))
            return false;
        for (auto &robotAABB : robotAABBs)
            if (checkObstacles(robotAABB, &result))
                return false;
    }
    if (request.checkObstacle) {
        auto trafo = m_robots[0]->getTransformation(config);
        AABB robotAABB = util::transformAABB(m_robotAABBs[0], trafo);
        return !checkObstacles(robotAABB, &result);
    }

    return true;
}

/*!
*  \brief      Check collision of a trajectory of configurations
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionDetectionAABB<dim>::check(const std::vector<Vector<dim>> &configs) const {
    if (configs.empty())
        return false;

    for (auto &config : configs)
        if (!check(config))
            return false;

    return true;
}

template <unsigned int dim>
bool CollisionDetectionAABB<dim>::checkRobots(const std::vector<AABB> &robots, CollisionResult *result) const {
    if (result) {
        double dist;
        for (auto a = robots.begin(); a != robots.end() - 1; ++a) {
            for (auto b = robots.begin() + 1; b != robots.end(); ++b) {
                dist = std::sqrt(a->squaredExteriorDistance(*b));
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
                if (a->intersects(*b))
                    return true;
    }

    return false;
}

template <unsigned int dim>
bool CollisionDetectionAABB<dim>::checkObstacles(const AABB &robotAABB, CollisionResult *result) const {
    if (result) {
        double dist;
        for (auto &obstacle : m_obstacleAABBs) {
            dist = std::sqrt(robotAABB.squaredExteriorDistance(obstacle));
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
            if (robotAABB.intersects(obstacle))
                return true;
    }

    return false;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONAABB_HPP */
