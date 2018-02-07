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
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr);
    bool checkTrajectory(const std::vector<Vector<dim>> &configs);

  private:
    bool check(const Vector<dim> &config, const CollisionRequest &request);

    std::vector<std::shared_ptr<FCLModel>> m_robotModels;

    using CollisionFcl<dim>::m_environment;
    using CollisionFcl<dim>::m_identity;
    using CollisionFcl<dim>::m_workspaceBounding;
    using CollisionFcl<dim>::m_obstacles;
    using CollisionFcl<dim>::m_workspaceAvaible;
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
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionFclMobile<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *request, CollisionResult *result) {
    CollisionRequest collisionRequest = this->m_request;
    if (request)
        collisionRequest = *request;

    return check(config, collisionRequest);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionFclMobile<dim>::checkTrajectory(const std::vector<Vector<dim>> &configs) {
    if (configs.empty())
        return false;

    CollisionResult result;
    for (auto &config : configs)
        if (check(config, m_request))
            return true;

    return false;
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionFclMobile<dim>::check(const Vector<dim> &config, const CollisionRequest &request) {
    auto configVecs = util::splitVec<dim>(config, m_environment->getRobotDimSizes());

    if (request.checkInterRobot) {    // inter robot collisions
        for (size_t i = 0; i < m_robotModels.size() - 1; ++i) {
            for (size_t j = i + 1; j < m_robotModels.size(); ++j) {
                Transform T1 = m_environment->getRobot()->getTransformation(configVecs[i]);
                Transform T2 = m_environment->getRobot()->getTransformation(configVecs[j]);
                if (checkFCL(m_robotModels[i], m_robotModels[j], T1, T2)) {
                    Logging::trace("Collision between robot " + m_environment->getRobots()[i]->getName() + " and robot " + m_environment->getRobots()[j]->getName(), this);
                    if (!request.completeCheck)
                        return true;
                }
            }
        }
    }

    if (request.checkObstacle && m_workspaceAvaible) {    // collisions between robots and obstacles
        for (size_t i = 0; i < m_robotModels.size(); ++i) {
            Transform T = m_environment->getRobot()->getTransformation(configVecs[i]);
            for (auto &obstacle : m_obstacles) {
                if (checkFCL(m_robotModels[i], obstacle.first, T, obstacle.second)) {
                    Logging::trace("Collision between robot " + std::to_string(i) + " and obstacle", this);
                    if (!request.completeCheck)
                        return true;
                }
            }
        }
    }
    return false;
}

} /* namespace ippp */

#endif /* COLLISIONFCLMOBILE_HPP */
