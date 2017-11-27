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

#ifndef COLLISIONDETECTIONPQP_HPP
#define COLLISIONDETECTIONPQP_HPP

#include <ippp/environment/model/ModelPqp.h>
#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/util/UtilCollision.hpp>

namespace ippp {

/*!
* \brief   Class collision detection with the pqp library
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionPqp : public CollisionDetection<dim> {
  public:
    CollisionDetectionPqp(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr) override;
    bool checkTrajectory(std::vector<Vector<dim>> &configs) override;

  protected:
    bool checkSerialRobot(const Vector<dim> &config);
    bool checkMobileRobot(const Vector<dim> &config);

    bool checkPQP(PQP_Model *model1, PQP_Model *model2, const Transform &T1, const Transform &T2);

    Transform m_identity;
    AABB m_workspaceBounding;
    std::vector<PQP_Model *> m_obstacles;
    bool m_workspaceAvaible = false;

    PQP_Model *m_baseModel = nullptr;
    bool m_baseMeshAvaible = false;
    std::vector<PQP_Model *> m_jointModels;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionPqp
*  \details    Copies all the models of the robot local to reduce calls.
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionPqp<dim>::CollisionDetectionPqp(const std::shared_ptr<Environment> &environment,
                                                  const CollisionRequest &request)
    : CollisionDetection<dim>("CollisionDetectionPQP", environment, request) {
    m_identity = Transform::Identity();
    auto robot = m_environment->getRobot();
    this->setRobotBoundings(m_environment->getRobotBoundaries());
    m_workspaceBounding = environment->getSpaceBoundary();

    if (robot->getBaseModel() != nullptr && !robot->getBaseModel()->empty()) {
        m_baseModel = &std::static_pointer_cast<ModelPqp>(robot->getBaseModel())->m_pqpModel;
        m_baseMeshAvaible = true;
    } else {
        Logging::error("Empty base model", this);
        return;
    }

    if (!environment->getObstacles().empty()) {
        for (auto &obstacle : environment->getObstacles()) {
            m_obstacles.push_back(&std::static_pointer_cast<ModelPqp>(obstacle)->m_pqpModel);
            m_workspaceAvaible = true;
        }
    } else {
        Logging::warning("No obstacles set", this);
    }

    if (robot->getRobotCategory() == RobotCategory::serial) {
        std::shared_ptr<SerialRobot> serialRobot(std::static_pointer_cast<SerialRobot>(robot));
        std::vector<std::shared_ptr<ModelContainer>> jointModels = serialRobot->getJointModels();
        if (!jointModels.empty()) {
            bool emptyJoint = false;
            for (const auto &model : jointModels) {
                if (!model || model->empty()) {
                    emptyJoint = true;
                }
            }
            if (!emptyJoint) {
                for (unsigned int i = 0; i < dim; ++i) {
                    m_jointModels.push_back(&std::static_pointer_cast<ModelPqp>(serialRobot->getModelFromJoint(i))->m_pqpModel);
                }
            } else {
                Logging::error("Emtpy joint model", this);
            }
        } else {
            Logging::error("No joint models applied", this);
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
bool CollisionDetectionPqp<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *request, CollisionResult *result) {
    if (m_environment->getRobot()->getRobotCategory() == RobotCategory::mobile)
        return checkMobileRobot(config);

        return checkSerialRobot(config);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkTrajectory(std::vector<Vector<dim>> &configs) {
    if (configs.empty())
        return false;

    if (m_environment->getRobot()->getRobotCategory() == RobotCategory::mobile) {
        for (size_t i = 0; i < configs.size(); ++i)
            if (checkMobileRobot(configs[i]))
                return true;
    } else {
        for (size_t i = 0; i < configs.size(); ++i)
            if (checkSerialRobot(configs[i]))
                return true;
    }
    return false;
}

/*!
*  \brief      Check for collision of a serial robot
*  \author     Sascha Kaden
*  \param[in]  configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkSerialRobot(const Vector<dim> &config) {
    if (this->checkRobotBounding(config))
        return true;

    auto robot = std::dynamic_pointer_cast<SerialRobot>(this->m_environment->getRobot());
    auto linkTrafos = robot->getLinkTrafos(config);
    auto pose = robot->getPose();

    // check models against workspace boundaries
    auto jointModels = robot->getJointModels();
    for (unsigned int i = 0; i < dim; ++i)
        if (!m_workspaceBounding.contains(util::transformAABB(jointModels[i]->m_mesh.aabb, linkTrafos[i])))
            return true;

    // control collision of the robot joints with themselves
    if (m_baseMeshAvaible)
        for (unsigned int i = 1; i < dim; ++i)
            if (checkPQP(m_baseModel, m_jointModels[i], pose, linkTrafos[i]))
                return true;

    for (unsigned int i = 0; i < dim; ++i)
        for (unsigned int j = i + 2; j < dim; ++j)
            if (checkPQP(m_jointModels[i], m_jointModels[j], linkTrafos[i], linkTrafos[j]))
                return true;

    // control collision with workspace
    if (m_workspaceAvaible) {
        for (auto &obstacle : m_obstacles)
            if (checkPQP(obstacle, m_baseModel, m_identity, pose))
                return true;

        for (unsigned int i = 0; i < dim; ++i)
            for (auto &obstacle : m_obstacles)
                if (checkPQP(obstacle, m_jointModels[i], m_identity, linkTrafos[i]))
                    return true;
    }
    return false;
}

/*!
*  \brief      Check for collision of a mobile robot (mesh with workspace)
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkMobileRobot(const Vector<dim> &config) {
    if (this->checkRobotBounding(config))
        return true;

    auto T = m_environment->getRobot()->getTransformation(config);

    if (m_baseMeshAvaible && m_workspaceAvaible) {
        for (auto obstacle : m_obstacles) {
            if (checkPQP(obstacle, m_baseModel, m_identity, T)) {
                return true;
            }
        }
    }
    return false;
}

/*!
*  \brief      Check for collision with PQP library
*  \author     Sascha Kaden
*  \param[in]  PQP mesh model one
*  \param[in]  PQP mesh model two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2016-07-14
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkPQP(PQP_Model *model1, PQP_Model *model2, const Transform &T1, const Transform &T2) {
    PQP_REAL pqpR1[3][3], pqpR2[3][3], pqpT1[3], pqpT2[3];

    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            pqpR1[i][j] = T1.linear()(i, j);
            pqpR2[i][j] = T2.linear()(i, j);
        }
        pqpT1[i] = T1.translation()(i);
        pqpT2[i] = T2.translation()(i);
    }

    PQP_CollideResult cres;
    PQP_Collide(&cres, pqpR1, pqpT1, model1, pqpR2, pqpT2, model2, PQP_FIRST_CONTACT);
    return cres.NumPairs() > 0;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONPQP_HPP */
