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

#ifndef COLLISIONFCLMOBILE_HPP
#define COLLISIONFCLMOBILE_HPP

#include <ippp/modules/collisionDetection/CollisionFcl.hpp>

namespace ippp {

/*!
* \brief   Class for collision detection with the fcl library and mobile robots
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionFclMobile : public CollisionFcl<dim> {
  public:
    CollisionFclMobile(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());

    bool check(const Vector<dim> &config) const;
    bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  private:
    bool check(const Vector<dim> &config, const CollisionRequest &request) const;

    std::vector<std::shared_ptr<FCLModel>> m_robotModels;
    std::vector<std::shared_ptr<MobileRobot>> m_robots;

    using CollisionDetection<dim>::m_request;
    using CollisionDetection<dim>::m_collisionCollector;
    using CollisionFcl<dim>::m_environment;
    using CollisionFcl<dim>::m_identity;
    using CollisionFcl<dim>::m_workspaceBounding;
    using CollisionFcl<dim>::m_obstacles;
    using CollisionFcl<dim>::m_obstacleExists;
};

/*!
*  \brief      Constructor of the class CollisionFclMobile
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionRequest
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionFclMobile<dim>::CollisionFclMobile(const std::shared_ptr<Environment> &environment, const CollisionRequest &request)
    : CollisionFcl<dim>("FCL mobile", environment, request) {
    auto robots = m_environment->getRobots();

    for (auto &robot : robots) {
        if (robot->getBaseModel() != nullptr && !robot->getBaseModel()->empty()) {
            m_robots.push_back(std::dynamic_pointer_cast<MobileRobot>(robot));
            m_robotModels.push_back(std::static_pointer_cast<ModelFcl>(robot->getBaseModel())->m_fclModel);
        } else {
            Logging::warning("Empty base model", this);
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
bool CollisionFclMobile<dim>::check(const Vector<dim> &config) const {
    return check(config, m_request);
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
bool CollisionFclMobile<dim>::check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const {
    return check(config, request);
}

/*!
*  \brief      Check collision of a trajectory of configurations
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if valid)
*  \date       2018-02-12
*/
template <unsigned int dim>
bool CollisionFclMobile<dim>::check(const std::vector<Vector<dim>> &configs) const {
    if (configs.empty())
        return false;

    CollisionResult result;
    for (auto &config : configs)
        if (!check(config, m_request))
            return false;

    return true;
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if valid)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionFclMobile<dim>::check(const Vector<dim> &config, const CollisionRequest &request) const {
    m_collisionCollector->add(1);
    auto configVecs = util::splitVec<dim>(config, m_environment->getRobotDimSizes());

    if (request.checkInterRobot) {    // inter robot collisions
        for (size_t i = 0; i < m_robotModels.size() - 1; ++i) {
            for (size_t j = i + 1; j < m_robotModels.size(); ++j) {
                Transform T1 = m_robots[i]->getTransformation(configVecs[i]);
                Transform T2 = m_robots[j]->getTransformation(configVecs[j]);
                if (this->checkFCL(m_robotModels[i], m_robotModels[j], T1, T2)) {
                    // Logging::trace("Collision between robot " + m_robots[i]->getName() + " and robot " +
                    // m_robots[j]->getName(), this);
                    if (!request.completeCheck)
                        return false;
                }
            }
        }
    }

    if (request.checkObstacle && m_obstacleExists) {    // collisions between robots and obstacles
        for (size_t i = 0; i < m_robotModels.size(); ++i) {
            Transform T = m_robots[i]->getTransformation(configVecs[i]);
            for (auto &obstacle : m_obstacles) {
                if (this->checkFCL(m_robotModels[i], obstacle.first, T, obstacle.second)) {
                    // Logging::trace("Collision between robot " + std::to_string(i) + " and obstacle", this);
                    if (!request.completeCheck)
                        return false;
                }
            }
        }
    }
    return true;
}

} /* namespace ippp */

#endif /* COLLISIONFCLMOBILE_HPP */
