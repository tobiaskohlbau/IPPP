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

    bool check(const Vector<dim> &config) const;
    bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const;
    bool check(const std::vector<Vector<dim>> &configs) const;

  private:
    bool check(const Vector<dim> &config, const CollisionRequest &request) const;

    bool m_baseModelExists = false;
    bool m_toolModelExists = false;
    std::shared_ptr<FCLModel> m_baseModel;
    std::shared_ptr<FCLModel> m_toolModel;
    std::vector<std::shared_ptr<FCLModel>> m_linkModels;
    AABB m_toolAABB;
    std::vector<AABB> m_linkAABBs;
    std::shared_ptr<SerialRobot> m_robot;

    using CollisionDetection<dim>::m_request;
    using CollisionDetection<dim>::m_collisionCollector;
    using CollisionFcl<dim>::m_environment;
    using CollisionFcl<dim>::m_identity;
    using CollisionFcl<dim>::m_workspaceBounding;
    using CollisionFcl<dim>::m_obstacles;
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
    if (m_environment->getRobot()->getRobotCategory() != RobotCategory::serial) {
        Logging::error("Robot has to be serial!", this);
        return;
    }
    m_robot = std::static_pointer_cast<SerialRobot>(m_environment->getRobot());

    if (m_robot->getBaseModel() != nullptr && !m_robot->getBaseModel()->empty()) {
        m_baseModel = std::static_pointer_cast<ModelFcl>(m_robot->getBaseModel())->m_fclModel;
        m_baseModelExists = true;
    } else {
        Logging::debug("Empty base model", this);
    }

    if (m_robot->getToolModel() != nullptr && !m_robot->getToolModel()->empty()) {
        auto model = std::static_pointer_cast<ModelFcl>(m_robot->getToolModel());
        m_toolModel = model->m_fclModel;
        m_toolAABB = model->getAABB();
        m_toolModelExists = true;
    } else {
        Logging::debug("Empty tool model", this);
    }

    auto linkModels = m_robot->getLinkModels();
    if (linkModels.empty())
        Logging::error("No link models applied", this);

    for (auto &model : linkModels) {
        if (!model || model->empty()) {
            Logging::error("Emtpy link model", this);
            return;
        }
        auto fclModel = std::static_pointer_cast<ModelFcl>(model);
        m_linkModels.push_back(fclModel->m_fclModel);
        m_linkAABBs.push_back(fclModel->getAABB());
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
bool CollisionFclSerial<dim>::check(const Vector<dim> &config) const {
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
bool CollisionFclSerial<dim>::check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const {
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
bool CollisionFclSerial<dim>::check(const std::vector<Vector<dim>> &configs) const {
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
bool CollisionFclSerial<dim>::check(const Vector<dim> &config, const CollisionRequest &request) const {
    if (!this->checkRobotBound(config))
        return false;

    m_collisionCollector->add(1);
    auto linkAndToolTrafos = m_robot->getLinkAndToolTrafos(config);
    auto &linkTrafos = linkAndToolTrafos.first;
    auto &toolTrafo = linkAndToolTrafos.second;
    auto pose = m_robot->getPose();

    // check models against workspace boundaries
    for (unsigned int i = 0; i < dim; ++i) {
        if (!m_workspaceBounding.contains(util::transformAABB(m_linkAABBs[i], linkTrafos[i]))) {
            // Logging::trace("Robot out of workspace boundaries", this);
            return false;
        }
    }
    if (m_toolModelExists && !m_workspaceBounding.contains(util::transformAABB(m_toolAABB, toolTrafo))) {
        // Logging::trace("Robot out of workspace boundaries", this);
        return false;
    }

    // control collision of the robot joints with themselves
    if (request.checkInterRobot) {
        if (m_baseModelExists) {
            for (unsigned int i = 1; i < dim; ++i) {
                if (this->checkFCL(m_baseModel, m_linkModels[i], pose, linkTrafos[i])) {
                    // std::cout << pose.matrix() << std::endl;
                    // std::cout << linkTrafos[i].matrix() << std::endl;
                    // Logging::trace("Collision between link" + std::to_string(i) + " and base", this);
                    return false;
                }
            }
        }
        for (unsigned int i = 0; i < dim; ++i) {
            for (unsigned int j = i + 2; j < dim; ++j) {
                if (this->checkFCL(m_linkModels[i], m_linkModels[j], linkTrafos[i], linkTrafos[j])) {
                    // Logging::trace("Collision between link" + std::to_string(i) + " and link" + std::to_string(j), this);
                    // std::cout << linkTrafos[i].matrix() << std::endl;
                    // std::cout << linkTrafos[j].matrix() << std::endl;
                    return false;
                }
            }
        }
        if (m_toolModelExists) {
            for (unsigned int i = 0; i < dim - 1; ++i) {
                if (this->checkFCL(m_toolModel, m_linkModels[i], toolTrafo, linkTrafos[i])) {
                    // std::cout << toolTrafo.matrix() << std::endl;
                    // std::cout << linkTrafos[i].matrix() << std::endl;
                    // Logging::trace("Collision between link" + std::to_string(i) + " and tool", this);
                    return false;
                }
            }
        }
    }

    // control collision with workspace
    if (request.checkObstacle) {
        auto trafo = linkTrafos.begin();
        for (auto model = m_linkModels.begin(); model < m_linkModels.end(); ++model, ++trafo) {
            for (auto &obstacle : m_obstacles) {
                if (this->checkFCL(obstacle.first, *model, obstacle.second, *trafo)) {
                    return false;
                }
            }
        }
        if (m_toolModelExists) {
            for (auto &obstacle : m_obstacles) {
                if (this->checkFCL(obstacle.first, m_toolModel, obstacle.second, toolTrafo)) {
                    // Logging::trace("Collision between workspace and tool", this);
                    return false;
                }
            }
        }
    }
    return true;
}

} /* namespace ippp */

#endif /* COLLISIONFCLSERIAL_HPP */
