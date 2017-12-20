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

#ifndef COLLISIONDETECTIONFCL_HPP
#define COLLISIONDETECTIONFCL_HPP

#include <fcl/BVH/BVH_model.h>
#include <fcl/collision.h>

#include <ippp/environment/model/ModelFcl.h>
#include <ippp/environment/robot/SerialRobot.h>
#include <ippp/modules/collisionDetection/CollisionDetection.hpp>
#include <ippp/util/UtilCollision.hpp>

namespace ippp {

/*!
* \brief   Class for collision detection with the fcl library
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionFcl : public CollisionDetection<dim> {
  public:
    CollisionDetectionFcl(const std::shared_ptr<Environment> &environment, const CollisionRequest &request = CollisionRequest());
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr);
    bool checkTrajectory(std::vector<Vector<dim>> &configs) override;

  private:
    bool checkSerialRobot(const Vector<dim> &config, const CollisionRequest &request);
    bool checkMobileRobot(const Vector<dim> &config, const CollisionRequest &request);
    bool checkFCL(const std::shared_ptr<FCLModel> &model1, const std::shared_ptr<FCLModel> &model2, const Transform &T1,
                  const Transform &T2);
    
    Transform m_identity;
    AABB m_workspaceBounding;
    std::vector<std::shared_ptr<FCLModel>> m_obstacles;
    bool m_workspaceAvaible = false;

    std::shared_ptr<FCLModel> m_baseModel;
    bool m_baseMeshAvaible = false;
    std::vector<std::shared_ptr<FCLModel>> m_jointModels;

    using CollisionDetection<dim>::m_environment;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionFcl<dim>::CollisionDetectionFcl(const std::shared_ptr<Environment> &environment,
                                                  const CollisionRequest &request)
    : CollisionDetection<dim>("FCL", environment, request) {
    m_identity = Transform::Identity();
    auto robot = m_environment->getRobot();
    this->setRobotBoundings(m_environment->getRobotBoundaries());
    m_workspaceBounding = environment->getSpaceBoundary();

    if (robot->getBaseModel() != nullptr && !robot->getBaseModel()->empty()) {
        m_baseModel =
            std::shared_ptr<FCLModel>(new FCLModel(std::static_pointer_cast<ModelFcl>(robot->getBaseModel())->m_fclModel));
        m_baseMeshAvaible = true;
    } else {
        Logging::warning("Empty base model", this);
    }

    if (!environment->getObstacles().empty()) {
        for (auto &obstacle : environment->getObstacles()) {
            m_obstacles.push_back(
                std::shared_ptr<FCLModel>(new FCLModel(std::static_pointer_cast<ModelFcl>(obstacle)->m_fclModel)));
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
            for (auto model : jointModels) {
                if (!model || model->empty()) {
                    emptyJoint = true;
                }
            }
            if (!emptyJoint) {
                for (unsigned int i = 0; i < dim; ++i) {
                    m_jointModels.push_back(std::shared_ptr<FCLModel>(
                        new FCLModel(std::static_pointer_cast<ModelFcl>(serialRobot->getModelFromJoint(i))->m_fclModel)));
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
bool CollisionDetectionFcl<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *request, CollisionResult *result) {
    CollisionRequest collisionRequest = this->m_request;
    if (request)
        collisionRequest = *request;

    if (m_environment->getRobot()->getRobotCategory() == RobotCategory::mobile)
        return checkMobileRobot(config, collisionRequest);
    else
        return checkSerialRobot(config, collisionRequest);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkTrajectory(std::vector<Vector<dim>> &configs) {
    if (configs.empty())
        return false;

    if (m_environment->getRobot()->getRobotCategory() == RobotCategory::mobile) {
        for (int i = 0; i < configs.size(); ++i)
            if (checkMobileRobot(configs[i], this->m_request))
                return true;
    } else {
        for (int i = 0; i < configs.size(); ++i)
            if (checkSerialRobot(configs[i], this->m_request))
                return true;
    }

    return false;
}

/*!
*  \brief      Check for collision of a serial robot
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkSerialRobot(const Vector<dim> &config, const CollisionRequest &request) {
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
            if (checkFCL(m_baseModel, m_jointModels[i], pose, linkTrafos[i]))
                return true;

    for (unsigned int i = 0; i < dim; ++i)
        for (unsigned int j = i + 2; j < dim; ++j)
            if (checkFCL(m_jointModels[i], m_jointModels[j], linkTrafos[i], linkTrafos[j]))
                return true;

    // control collision with workspace
    if (m_workspaceAvaible) {
        for (auto &obstacle : m_obstacles)
            if (checkFCL(obstacle, m_baseModel, m_identity, pose))
                return true;

        for (unsigned int i = 0; i < dim; ++i)
            for (auto &obstacle : m_obstacles)
                if (checkFCL(obstacle, m_jointModels[i], m_identity, linkTrafos[i]))
                    return true;
    }
    return false;
}

/*!
*  \brief      Check for collision of mobile robot (mesh with workspace)
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkMobileRobot(const Vector<dim> &config, const CollisionRequest &request) {
    if (this->checkRobotBounding(config))
        return true;

    Transform T = m_environment->getRobot()->getTransformation(config);

    if (m_baseMeshAvaible && m_workspaceAvaible) {
        for (auto obstacle : m_obstacles)
            if (checkFCL(obstacle, m_baseModel, m_identity, T))
                return true;
    }
    return false;
}

/*!
*  \brief      Check for collision with FCL library
*  \author     Sascha Kaden
*  \param[in]  FCL mesh model one
*  \param[in]  FCL mesh model two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkFCL(const std::shared_ptr<FCLModel> &model1, const std::shared_ptr<FCLModel> &model2,
                                          const Transform &T1, const Transform &T2) {
    const Eigen::Matrix3f R1 = T1.rotation().cast<float>();
    const Eigen::Vector3f t1 = T1.translation().cast<float>();
    fcl::Matrix3f fclR1(R1(0), R1(1), R1(2), R1(3), R1(4), R1(5), R1(6), R1(7), R1(8));
    fcl::Vec3f fclT1(t1(0), t1(1), t1(2));

    const Eigen::Matrix3f R2 = T2.rotation().cast<float>();
    const Eigen::Vector3f t2 = T2.translation().cast<float>();
    fcl::Matrix3f fclR2(R2(0), R2(1), R2(2), R2(3), R2(4), R2(5), R2(6), R2(7), R2(8));
    fcl::Vec3f fclT2(t2(0), t2(1), t2(2));

    fcl::CollisionObject o1(model1, fclR1, fclT1);
    fcl::CollisionObject o2(model2, fclR2, fclT2);
    fcl::CollisionRequest request;    // default setting
    fcl::CollisionResult result;
    fcl::collide(&o1, &o2, request, result);

    return result.isCollision();
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONFCL_HPP */
