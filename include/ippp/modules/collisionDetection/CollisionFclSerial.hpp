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

#ifndef COLLISIONFCLSERIAL_HPP
#define COLLISIONFCLSERIAL_HPP

#include <ippp/modules/collisionDetection/CollisionFcl.hpp>

namespace ippp {

/*!
* \brief   Class for collision detection with the fcl library and serial robots
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionFclSerial : public CollisionFcl<dim> {
  public:
    CollisionFclSerial(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr);
    bool checkTrajectory(const std::vector<Vector<dim>> &configs);

  private:
    bool check(const Vector<dim> &config, const CollisionRequest &request);

    std::shared_ptr<FCLModel> m_baseModel;
    bool m_baseMeshAvaible = false;
    std::vector<std::shared_ptr<FCLModel>> m_linkModels;

    using CollisionDetection<dim>::m_request;
    using CollisionFcl<dim>::m_environment;
    using CollisionFcl<dim>::m_identity;
    using CollisionFcl<dim>::m_workspaceBounding;
    using CollisionFcl<dim>::m_obstacles;
    using CollisionFcl<dim>::m_workspaceAvaible;
};

/*!
*  \brief      Constructor of the class CollisionFclSerial
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \param[in]  CollisionRequest
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionFclSerial<dim>::CollisionFclSerial(const std::shared_ptr<Environment> &environment, const CollisionRequest &request)
    : CollisionFcl<dim>("FCL Serial", environment, request) {
    auto robot = m_environment->getRobot();

    if (robot->getBaseModel() != nullptr && !robot->getBaseModel()->empty()) {
        m_baseModel = std::static_pointer_cast<ModelFcl>(robot->getBaseModel())->m_fclModel;
        m_baseMeshAvaible = true;
    } else {
        Logging::warning("Empty base model", this);
    }

    if (robot->getRobotCategory() == RobotCategory::serial) {
        std::shared_ptr<SerialRobot> serialRobot(std::static_pointer_cast<SerialRobot>(robot));
        std::vector<std::shared_ptr<ModelContainer>> linkModels = serialRobot->getLinkModels();
        if (linkModels.empty())
            Logging::error("No link models applied", this);

        for (auto &model : linkModels) {
            if (!model || model->empty())
                Logging::error("Emtpy joint model", this);
            else
                m_linkModels.push_back(std::static_pointer_cast<ModelFcl>(model)->m_fclModel);
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
bool CollisionFclSerial<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *request, CollisionResult *result) {
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
bool CollisionFclSerial<dim>::checkTrajectory(const std::vector<Vector<dim>> &configs) {
    if (configs.empty())
        return false;

    for (int i = 0; i < configs.size(); ++i)
        if (check(configs[i], m_request))
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
bool CollisionFclSerial<dim>::check(const Vector<dim> &config, const CollisionRequest &request) {
    auto robot = std::dynamic_pointer_cast<SerialRobot>(this->m_environment->getRobot());
    auto linkTrafos = robot->getLinkTrafos(config);
    auto pose = robot->getPose();

    // check models against workspace boundaries
    auto linkModels = robot->getLinkModels();
    for (unsigned int i = 0; i < dim; ++i) {
        if (!m_workspaceBounding.contains(util::transformAABB(linkModels[i]->m_mesh.aabb, linkTrafos[i]))) {
            // Logging::trace("Robot out of workspace boundaries", this);
            return true;
        }
    }

    // control collision of the robot joints with themselves
    if (request.checkInterRobot) {
        if (m_baseMeshAvaible) {
            for (unsigned int i = 1; i < dim; ++i) {
                if (this->checkFCL(m_baseModel, m_linkModels[i], pose, linkTrafos[i])) {
                    // std::cout << pose.matrix() << std::endl;
                    // std::cout << linkTrafos[i].matrix() << std::endl;
                    // Logging::trace("Collision between link" + std::to_string(i) + " and base", this);
                    return true;
                }
            }
        }

        for (unsigned int i = 0; i < dim; ++i) {
            for (unsigned int j = i + 2; j < dim; ++j) {
                if (this->checkFCL(m_linkModels[i], m_linkModels[j], linkTrafos[i], linkTrafos[j])) {
                    // Logging::trace("Collision between link" + std::to_string(i) + " and link" + std::to_string(j), this);
                    // std::cout << linkTrafos[i].matrix() << std::endl;
                    // std::cout << linkTrafos[j].matrix() << std::endl;
                    return true;
                }
            }
        }
    }

    // control collision with workspace
    if (m_workspaceAvaible && request.checkObstacle) {
        for (auto &obstacle : m_obstacles) {
            if (this->checkFCL(obstacle.first, m_baseModel, obstacle.second, pose)) {
                // Logging::trace("Collision between workspace and base", this);
                return true;
            }
        }

        for (unsigned int i = 0; i < dim; ++i) {
            for (auto &obstacle : m_obstacles) {
                if (this->checkFCL(obstacle.first, m_linkModels[i], obstacle.second, linkTrafos[i])) {
                    // Logging::trace("Collision between workspace and link" + std::to_string(i), this);
                    return true;
                }
            }
        }
    }
    return false;
}

} /* namespace ippp */

#endif /* COLLISIONFCLSERIAL_HPP */
