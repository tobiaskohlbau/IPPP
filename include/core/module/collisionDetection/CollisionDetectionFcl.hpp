//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef COLLISIONDETECTIONFCL_H_
#define COLLISIONDETECTIONFCL_H_

#include <core/module/collisionDetection/CollisionDetection.hpp>
#include <core/utility/UtilCollision.hpp>
#include <robot/SerialRobot.hpp>

namespace rmpl {

/*!
* \brief   Class for collision detection with the fcl library
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionFcl : public CollisionDetection<dim> {
  public:
    CollisionDetectionFcl(const std::shared_ptr<RobotBase<dim>> &robot);
    bool controlVec(const Vector<dim> &vec);
    bool controlTrajectory(std::vector<Vector<dim>> &vec);

  private:
    bool controlCollisionMesh(const Vector<dim> &vec);
    bool checkSerialRobot(const Vector<dim> &vec);
    bool checkMobileRobot(const Vector<dim> &vec);
    bool checkMesh(std::vector<FCLModel> &models, FCLModel &base, Matrix3 R[],
                   Matrix3 &poseR, Vector3 t[], Vector3 &poseT);
    bool checkFCL(FCLModel &model1, FCLModel &model2, Matrix3 &R1, Matrix3 &R2,
                  Vector3 &t1, Vector3 &t2);


    // models for collision detection
    fcl::CollisionObject<float> *o1 = nullptr;
    fcl::CollisionObject<float> *o2 = nullptr;

    Matrix3 m_identity;
    Eigen::Vector3f m_zeroVec;
    using CollisionDetection<dim>::m_robot;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionFcl<dim>::CollisionDetectionFcl(const std::shared_ptr<RobotBase<dim>> &robot) : CollisionDetection<dim>("collisionDetection", robot) {
    m_identity = Matrix3::Identity(3, 3);
    m_zeroVec = Eigen::Vector3f::Zero(3, 1);
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::controlVec(const Vector<dim> &vec) {
    return controlCollisionMesh(vec);
}

/*!
*  \brief      Differentiate between mobile and serial robot
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::controlCollisionMesh(const Vector<dim> &vec) {
    if (m_robot->getRobotType() == RobotType::mobile)
        return checkMobileRobot(vec);
    else
        return checkSerialRobot(vec);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] binary result of collision (true if in collision)
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::controlTrajectory(std::vector<Vector<dim>> &vecs) {
    if (vecs.size() == 0)
        return false;

    if (m_robot->getRobotType() == RobotType::mobile) {
        for (int i = 0; i < vecs.size(); ++i)
            if (checkMobileRobot(vecs[i]))
                return true;
    } else {
        for (int i = 0; i < vecs.size(); ++i)
            if (checkSerialRobot(vecs[i]))
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
bool CollisionDetectionFcl<dim>::checkSerialRobot(const Vector<dim> &vec) {
    std::shared_ptr<SerialRobot<dim>> robot(std::static_pointer_cast<SerialRobot<dim>>(m_robot));
    Matrix3 poseR;
    Vector3 poseT;
    Matrix3 rot[dim];
    Vector3 trans[dim];
    utilCollision::getTrafosFromRobot<dim>(vec, robot, poseR, poseT, rot, trans);

    FCLModel baseMesh;
    if (robot->getBaseMesh() != nullptr)
        baseMesh = robot->getBaseMesh()->m_fclModel;
    std::vector<FCLModel> jointMeshes = robot->getJointModels();
    return checkMesh(jointMeshes, baseMesh, rot, poseR, trans, poseT);
}

/*!
*  \brief      Check for collision of mobile robot (mesh with workspace)
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkMobileRobot(const Vector<dim> &vec) {
        Matrix4 pose = m_robot->getPoseMat();
        Matrix3 poseR;
        Vector3 poseT;
        utilGeo::decomposeT(pose, poseR, poseT);

    if (m_robot->getBaseMesh() == nullptr || m_robot->getWorkspace() == nullptr)
        return false;

    FCLModel baseMesh = m_robot->getBaseMesh()->m_fclModel;
    FCLModel workspace = m_robot->getWorkspace()->getFcl();

    return checkFCL(workspace, baseMesh, m_identity, poseR, m_zeroVec, poseT);
}

/*!
*  \brief      Check for collision with fcl library
*  \author     Sascha Kaden
*  \param[in]  MeshContainer one
*  \param[in]  MeshContainer two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionFcl<dim>::checkMesh(std::vector<FCLModel> &models, FCLModel &baseModel,
                                        Matrix3 R[], Matrix3 &poseR, Vector3 t[],
                                        Vector3 &poseT) {
    // control collision between baseModel and joints
    if (baseModel.num_vertices != 0) {
        for (int i = 1; i < dim; ++i) {
            if (checkFCL(baseModel, models[i], poseR, R[i], poseT, t[i]))
                return true;
        }
    }

    // control collision of the robot joints with themself
    for (int i = 0; i < dim; ++i) {
        for (int j = i + 2; j < dim; ++j) {
            if (checkFCL(models[i], models[j], R[i], R[j], t[i], t[j])) {
                if (Logging::getLogLevel() == LogLevel::debug) {
                    Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                    Eigen::Vector3f r = R[i].eulerAngles(0, 1, 2);
                    std::cout << "A" << i << ": ";
                    std::cout << "Euler angles: " << r.transpose() << "\t";
                    std::cout << "Translation: " << t[i].transpose() << std::endl;
                    r = R[j].eulerAngles(0, 1, 2);
                    std::cout << "A" << j << ": ";
                    std::cout << "Euler angles: " << r.transpose() << "\t";
                    std::cout << "Translation: " << t[j].transpose() << std::endl << std::endl;
                }
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_robot->getWorkspace() != nullptr) {
        FCLModel workspace = m_robot->getWorkspace()->getFcl();

        if (checkFCL(workspace, baseModel, m_identity, poseR, m_zeroVec, poseT))
            return true;

        for (int i = 0; i < dim; ++i) {
            if (checkFCL(workspace, models[i], m_identity, R[i], m_zeroVec, t[i])) {
                Logging::debug("Collision between workspace and link " + std::to_string(i), this);
                return true;
            }
        }
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
bool CollisionDetectionFcl<dim>::checkFCL(FCLModel &model1, FCLModel &model2, Matrix3 &R1,
                                       Matrix3 &R2, Vector3 &t1, Vector3 &t2) {
    o1 = new fcl::CollisionObject<float>(&model1, R1, t1);
    o2 = new fcl::CollisionObject<float>(&model2, R2, t2);
    fcl::CollisionRequest<float> request;    // default setting
    fcl::CollisionResult<float> result;
    collide(o1, o2, request, result);
    delete o1;
    delete o2;

    return result.isCollision();
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTIONFCL_H_ */
