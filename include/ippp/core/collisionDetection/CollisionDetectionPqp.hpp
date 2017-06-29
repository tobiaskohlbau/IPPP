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

#include <ippp/core/collisionDetection/CollisionDetection.hpp>
#include <ippp/core/util/UtilCollision.hpp>
#include <ippp/environment/model/ModelPqp.h>
#include <ippp/environment/robot/SerialRobot.h>

namespace ippp {

/*!
* \brief   Class collision detection with the pqp library
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionPqp : public CollisionDetection<dim> {
  public:
    CollisionDetectionPqp(const std::shared_ptr<Environment> &environment);
    bool checkConfig(const Vector<dim> &config, CollisionData *data = nullptr) override;
    bool checkTrajectory(std::vector<Vector<dim>> &configs) override;

  protected:
    bool checkSerialRobot(const Vector<dim> &vec);
    bool checkMobileRobot(const Vector<dim> &vec);
    bool checkMesh(std::vector<Matrix3> Rs, Matrix3 &poseR, std::vector<Vector3> ts, Vector3 &poseT);

    bool checkPQP(PQP_Model *model1, PQP_Model *model2, Matrix3 &R1, Matrix3 &R2, Vector3 &t1, Vector3 &t2);

    Matrix3 m_identity;
    Vector3 m_zeroVec;

    PQP_Model *m_baseMesh = nullptr;
    std::vector<PQP_Model *> m_obstacles;
    bool m_baseMeshAvaible = false;
    bool m_workspaceAvaible = false;
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
CollisionDetectionPqp<dim>::CollisionDetectionPqp(const std::shared_ptr<Environment> &environment)
    : CollisionDetection<dim>("CollisionDetectionPQP", environment) {
    m_identity = Matrix3::Identity(3, 3);
    m_zeroVec = Vector3::Zero(3, 1);

    auto robot = m_environment->getRobot();

    if (robot->getBaseModel() != nullptr && !robot->getBaseModel()->empty()) {
        m_baseMesh = &std::static_pointer_cast<ModelPqp>(robot->getBaseModel())->m_pqpModel;
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
    }

    if (robot->getRobotType() == RobotType::serial) {
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
                for (int i = 0; i < dim; ++i) {
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
bool CollisionDetectionPqp<dim>::checkConfig(const Vector<dim> &config, CollisionData *data) {
    if (m_environment->getRobot()->getRobotType() == RobotType::mobile) {
        return checkMobileRobot(config);
    } else {
        return checkSerialRobot(config);
    }
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
    if (configs.empty()) {
        return false;
    }

    if (m_environment->getRobot()->getRobotType() == RobotType::mobile) {
        for (int i = 0; i < configs.size(); ++i) {
            if (checkMobileRobot(configs[i])) {
                return true;
            }
        }
    } else {
        for (int i = 0; i < configs.size(); ++i) {
            if (checkSerialRobot(configs[i])) {
                return true;
            }
        }
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
bool CollisionDetectionPqp<dim>::checkSerialRobot(const Vector<dim> &vec) {
    auto robot = std::dynamic_pointer_cast<SerialRobot>(this->m_environment->getRobot());
    Matrix3 poseR;
    Vector3 poseT;
    std::pair<std::vector<Matrix3>, std::vector<Vector3>> rotAndTrans = util::getTrafosFromRobot(vec, robot, poseR, poseT);
    // for (auto tmp : jointTrafos)
    //    std::cout << tmp <<std::endl;
    // robot->saveMeshConfig(As);

    return checkMesh(rotAndTrans.first, poseR, rotAndTrans.second, poseT);
}

/*!
*  \brief      Check for collision of a mobile robot (mesh with workspace)
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkMobileRobot(const Vector<dim> &vec) {
    Matrix3 poseR;
    Vector3 poseT;
    util::poseVecToRandT(vec, poseR, poseT);

    if (m_baseMeshAvaible && m_workspaceAvaible) {
        for (auto obstacle : m_obstacles) {
            if (checkPQP(obstacle, m_baseMesh, m_identity, poseR, m_zeroVec, poseT)) {
                return true;
            }
        }
    } else {
        return false;
    }
}

/*!
*  \brief      Check for collision with the PQP library
*  \author     Sascha Kaden
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionPqp<dim>::checkMesh(std::vector<Matrix3> Rs, Matrix3 &poseR, std::vector<Vector3> ts, Vector3 &poseT) {
    // control collision between baseModel and joints
    if (m_baseMeshAvaible) {
        for (int i = 1; i < dim; ++i) {
            if (checkPQP(m_baseMesh, m_jointModels[i], poseR, Rs[i], poseT, ts[i])) {
                return true;
            }
        }
    }

    // control collision of the robot joints with themselves
    for (int i = 0; i < dim; ++i) {
        for (int j = i + 2; j < dim; ++j) {
            if (checkPQP(m_jointModels[i], m_jointModels[j], Rs[i], Rs[j], ts[i], ts[j])) {
                if (Logging::getLogLevel() == LogLevel::debug) {
                    auto temp = m_environment->getRobot();
                    std::shared_ptr<SerialRobot> robot(std::static_pointer_cast<SerialRobot>(temp));
                    Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                    Vector3 r = Rs[i].eulerAngles(0, 1, 2);
                    std::cout << "A" << i << ": ";
                    std::cout << "Euler angles: " << std::endl << Rs[i] << std::endl;
                    std::cout << "Translation: " << ts[i].transpose() << std::endl;
                    // robot->getMeshFromJoint(i)->saveObj(std::to_string(i) + ".obj", utilGeo::createT(R[i], t[i]));
                    r = Rs[j].eulerAngles(0, 1, 2);
                    std::cout << "A" << j << ": ";
                    std::cout << "Euler angles: " << std::endl << Rs[j] << std::endl;
                    std::cout << "Translation: " << ts[j].transpose() << std::endl << std::endl;
                    // robot->getMeshFromJoint(j)->saveObj(std::to_string(j) + ".obj", utilGeo::createT(R[i], t[i]));
                }
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_workspaceAvaible) {
        for (auto obstacle : m_obstacles) {
            if (checkPQP(obstacle, m_baseMesh, m_identity, poseR, m_zeroVec, poseT)) {
                return true;
            }
        }
        for (int i = 0; i < dim; ++i) {
            for (auto obstacle : m_obstacles) {
                if (checkPQP(obstacle, m_jointModels[i], m_identity, Rs[i], m_zeroVec, ts[i])) {
                    Logging::debug("Collision between workspace and link " + std::to_string(i), this);
                    return true;
                }
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
bool CollisionDetectionPqp<dim>::checkPQP(PQP_Model *model1, PQP_Model *model2, Matrix3 &R1, Matrix3 &R2, Vector3 &t1,
                                          Vector3 &t2) {
    PQP_REAL pqpR1[3][3], pqpR2[3][3], pqpT1[3], pqpT2[3];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pqpR1[i][j] = R1(i, j);
            pqpR2[i][j] = R2(i, j);
        }
        pqpT1[i] = t1(i);
        pqpT2[i] = t2(i);
    }

    PQP_CollideResult cres;
    PQP_Collide(&cres, pqpR1, pqpT1, model1, pqpR2, pqpT2, model2, PQP_FIRST_CONTACT);
    if (cres.NumPairs() > 0) {
        return true;
    } else {
        return false;
    }
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONPQP_HPP */
