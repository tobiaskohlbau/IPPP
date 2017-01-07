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

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <Eigen/Core>
#include <PQP.h>
#include <fcl/fcl.h>
#include <fcl/narrowphase/collision.h>

#include <core/dataObj/Node.hpp>
#include <core/dataObj/PointList.hpp>
#include <core/module/ModuleBase.h>
#include <robot/MeshContainer.h>
#include <robot/SerialRobot.hpp>
#include <robot/TriangleRobot2D.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return binary value
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class CollisionDetection : public ModuleBase {
  public:
    CollisionDetection(const std::shared_ptr<RobotBase<dim>> &robot);
    bool controlVec(const Vector<dim> &vec);
    bool controlTrajectory(std::vector<Vector<dim>> &vec);

  private:
    bool controlCollisionMesh(const Vector<dim> &vec);
    bool checkSerialRobot(const Vector<dim> &vec);
    bool checkMobileRobot(const Vector<dim> &vec);
    bool checkMesh(std::vector<std::shared_ptr<PQP_Model>> &models, std::shared_ptr<PQP_Model> &base, Matrix3 R[],
                   Matrix3 &poseR, Vector3 t[], Vector3 &poseT);
    bool checkMesh(std::vector<std::shared_ptr<FCLModel>> &models, std::shared_ptr<FCLModel> &base, Matrix3 R[],
                   Matrix3 &poseR, Vector3 t[], Vector3 &poseT);

    bool checkPQP(std::shared_ptr<PQP_Model> &model1, std::shared_ptr<PQP_Model> &model2, Matrix3 &R1,
                  Matrix3 &R2, Vector3 &t1, Vector3 &t2);
    bool checkFCL(std::shared_ptr<FCLModel> &model1, std::shared_ptr<FCLModel> &model2, Matrix3 &R1, Matrix3 &R2,
                  Vector3 &t1, Vector3 &t2);

    bool checkPoint2D(float x, float y);
    bool checkTriangleRobot(const Vector<dim> &vec);

    std::shared_ptr<RobotBase<dim>> m_robot;
    Vector<dim> m_minBoundary, m_maxBoundary;

    Eigen::MatrixXi m_2DWorkspace;
    std::shared_ptr<MeshContainer> m_workspace = nullptr;

    // models for collision detection
    fcl::CollisionObject<float> *o1;
    fcl::CollisionObject<float> *o2;

    Matrix3 m_identity;
    Eigen::Vector3f m_zeroVec;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::shared_ptr<RobotBase<dim>> &robot) : ModuleBase("CollisionDetection") {
    m_robot = robot;
    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();

    m_identity = Matrix3::Identity(3, 3);
    m_zeroVec = Eigen::Vector3f::Zero(3, 1);

    m_2DWorkspace = m_robot->get2DWorkspace();
    m_workspace = m_robot->getWorkspace();
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vec
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetection<dim>::controlVec(const Vector<dim> &vec) {

    if (m_robot->getCollisionType() == CollisionType::point2D)
        return checkPoint2D(vec[0], vec[1]);
    else if (m_robot->getCollisionType() == CollisionType::triangle2D)
        return checkTriangleRobot(vec);
    else
        return controlCollisionMesh(vec);
}

template <unsigned int dim>
bool CollisionDetection<dim>::controlCollisionMesh(const Vector<dim> &vec) {
    if (m_robot->getRobotType() == RobotType::mobile)
        return checkMobileRobot(vec);
    else
        return checkSerialRobot(vec);
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of points
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-05-25
*/
template <unsigned int dim>
bool CollisionDetection<dim>::controlTrajectory(std::vector<Vector<dim>> &vecs) {
    if (vecs.size() == 0)
        return false;

    if (m_robot->getCollisionType() == CollisionType::point2D) {
        for (int i = 0; i < vecs.size(); ++i)
            if (checkPoint2D(vecs[i][0], vecs[i][1]))
                return true;
    } else if (m_robot->getCollisionType() == CollisionType::triangle2D) {
        for (int i = 0; i < vecs.size(); ++i)
            if (checkTriangleRobot(vecs[i]))
                return true;
    } else {
        for (int i = 0; i < vecs.size(); ++i)
            if (controlCollisionMesh(vecs[i]))
                return true;
    }
    return false;
}

/*!
*  \brief      Check for 2D point collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] binary result of collision (true if in collision)
*  \date       2016-06-30
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkPoint2D(float x, float y) {
    if (m_minBoundary[0] >= x || x >= m_maxBoundary[0] || m_minBoundary[1] >= y || y >= m_maxBoundary[1]) {
        Logging::debug("Point out of workspace", this);
        return true;
    }

    if (m_2DWorkspace(x, y) < 80)
        return true;
    else
        return false;
}

/*!
*  \brief      Check for TriangleRobot collision
*  \author     Sascha Kaden
*  \param[in]  transformation Vector
*  \param[out] binary result of collision, true if in collision
*  \date       2016-12-19
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkTriangleRobot(const Vector<dim> &vec) {
    std::shared_ptr<TriangleRobot2D> robot(std::dynamic_pointer_cast<TriangleRobot2D>(m_robot));
    std::vector<Triangle2D> triangles = robot->getTriangles();
    VectorX vector = vec;
    vector.resize(3);

    Matrix2 R;
    Vector2 t;
    utilGeo::poseVecToRandT(vector, R, t);

    Vector2 u, temp;
    for (auto triangle : triangles) {
        triangle.transform(R, t);

        u = triangle.getP(2) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(2)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;

        u = triangle.getP(3) - triangle.getP(1);
        u /= u.norm();
        for (temp = triangle.getP(1) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;

        u = triangle.getP(3) - triangle.getP(2);
        u /= u.norm();
        for (temp = triangle.getP(2) + u; (temp - triangle.getP(3)).squaredNorm() > 2; temp += u)
            if (checkPoint2D(temp[0], temp[1]))
                return true;
    }

    return false;
}

/*!
*  \brief      Check for collision of serial robot
*  \author     Sascha Kaden
*  \param[in]  Vec of robot configuration
*  \param[out] binary result of collision
*  \date       2016-09-02
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkSerialRobot(const Vector<dim> &vec) {
    std::shared_ptr<SerialRobot<dim>> robot(std::static_pointer_cast<SerialRobot<dim>>(m_robot));

    std::vector<Matrix4> jointTrafos = robot->getJointTrafos(vec);
    Matrix4 pose = robot->getPoseMat();
    Matrix4 As[jointTrafos.size()];
    As[0] = pose * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i)
        As[i] = As[i - 1] * jointTrafos[i];

    Matrix3 poseR;
    Vector3 poseT;
    utilGeo::decomposeT(pose, poseR, poseT);

    Matrix3 rot[jointTrafos.size()];
    Vector3 trans[jointTrafos.size()];
    for (int i = 0; i < jointTrafos.size(); ++i)
        utilGeo::decomposeT(As[i], rot[i], trans[i]);

    if (m_robot->getCollisionType() == CollisionType::pqp) {
    std::shared_ptr<PQP_Model> baseMesh = nullptr;
        if (robot->getBaseMesh() != nullptr)
            baseMesh = robot->getBaseMesh()->m_pqpModel;
        std::vector<std::shared_ptr<PQP_Model>> jointMeshes = robot->getJointPqpModels();

        //        for (auto tmp : jointTrafos)
        //            std::cout << tmp <<std::endl;
        //        robot->saveMeshConfig(As);

        return checkMesh(jointMeshes, baseMesh, rot, poseR, trans, poseT);
    } else {
        std::shared_ptr<FCLModel> baseMesh = nullptr;
        if (robot->getBaseMesh() != nullptr)
            baseMesh = robot->getBaseMesh()->m_fclModel;
        std::vector<std::shared_ptr<FCLModel>> jointMeshes = robot->getJointFclModels();
        return checkMesh(jointMeshes, baseMesh, rot, poseR, trans, poseT);
    }
}

/*!
*  \brief      Check for collision of mobile robot (mesh with workspace)
*  \author     Sascha Kaden
*  \param[in]  Vec of robot configuration
*  \param[out] binary result of collision
*  \date       2016-11-14
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkMobileRobot(const Vector<dim> &vec) {
    Matrix4 pose = m_robot->getPoseMat();
    Matrix3 poseR;
    Vector3 poseT;
    utilGeo::decomposeT(pose, poseR, poseT);

    if (m_robot->getCollisionType() == CollisionType::pqp) {
        if (m_robot->getBaseMesh() == nullptr || m_robot->getWorkspace() == nullptr)
            return false;

        std::shared_ptr<PQP_Model> baseMesh = m_robot->getBaseMesh()->m_pqpModel;
        std::shared_ptr<PQP_Model> workspace = m_robot->getWorkspace()->getPqp();

        return checkPQP(workspace, baseMesh, m_identity, poseR, m_zeroVec, poseT);
    } else {
        if (m_robot->getBaseMesh() == nullptr || m_robot->getWorkspace() == nullptr)
            return false;

        std::shared_ptr<FCLModel> baseMesh = m_robot->getBaseMesh()->m_fclModel;
        std::shared_ptr<FCLModel> workspace = m_robot->getWorkspace()->getFcl();

        return checkFCL(workspace, baseMesh, m_identity, poseR, m_zeroVec, poseT);
    }
}

/*!
*  \brief      Check for collision with PQP or fcl library
*  \author     Sascha Kaden
*  \param[in]  MeshContainer one
*  \param[in]  MeshContainer two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2016-07-14
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkMesh(std::vector<std::shared_ptr<PQP_Model>> &models, std::shared_ptr<PQP_Model> &baseModel,
                                        Matrix3 R[], Matrix3 &poseR, Vector3 t[],
                                        Vector3 &poseT) {
    // control collision between baseModel and joints
    if (baseModel != nullptr) {
        for (int i = 1; i < dim; ++i) {
            if (checkPQP(baseModel, models[i], poseR, R[i], poseT, t[i]))
                return true;
        }
    }

    // control collision of the robot joints with themselves
    for (int i = 0; i < dim; ++i) {
        for (int j = i + 2; j < dim; ++j) {
            if (checkPQP(models[i], models[j], R[i], R[j], t[i], t[j])) {
                if (Logging::getLogLevel() == LogLevel::debug) {
                    std::shared_ptr<SerialRobot<dim>> robot(std::static_pointer_cast<SerialRobot<dim>>(m_robot));
                    Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                    Vector3 r = R[i].eulerAngles(0, 1, 2);
                    std::cout << "A" << i << ": ";
                    std::cout << "Euler angles: " << std::endl << R[i] << std::endl;
                    std::cout << "Translation: " << t[i].transpose() << std::endl;
                    robot->getMeshFromJoint(i)->saveObj(std::to_string(i) + ".obj", utilGeo::createT(R[i], t[i]));
                    r = R[j].eulerAngles(0, 1, 2);
                    std::cout << "A" << j << ": ";
                    std::cout << "Euler angles: " << std::endl << R[j] << std::endl;
                    std::cout << "Translation: " << t[j].transpose() << std::endl << std::endl;
                    robot->getMeshFromJoint(j)->saveObj(std::to_string(j) + ".obj", utilGeo::createT(R[i], t[i]));
                }
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_robot->getWorkspace() != nullptr) {
        std::shared_ptr<PQP_Model> workspace = m_robot->getWorkspace()->getPqp();

        if (checkPQP(workspace, baseModel, m_identity, poseR, m_zeroVec, poseT))
            return true;

        for (int i = 0; i < dim; ++i) {
            if (checkPQP(workspace, models[i], m_identity, R[i], m_zeroVec, t[i])) {
                Logging::debug("Collision between workspace and link " + std::to_string(i), this);
                return true;
            }
        }
    }
    return false;
}

/*!
*  \brief      Check for collision with PQP or fcl library
*  \author     Sascha Kaden
*  \param[in]  MeshContainer one
*  \param[in]  MeshContainer two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2016-07-14
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkMesh(std::vector<std::shared_ptr<FCLModel>> &models, std::shared_ptr<FCLModel> &baseModel,
                                        Matrix3 R[], Matrix3 &poseR, Vector3 t[],
                                        Vector3 &poseT) {
    // control collision between baseModel and joints
    if (baseModel != nullptr) {
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
        std::shared_ptr<FCLModel> workspace = m_robot->getWorkspace()->getFcl();

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
bool CollisionDetection<dim>::checkPQP(std::shared_ptr<PQP_Model> &model1, std::shared_ptr<PQP_Model> &model2, Matrix3 &R1,
                                       Matrix3 &R2, Vector3 &t1, Vector3 &t2) {
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
    PQP_Collide(&cres, pqpR1, pqpT1, model1.get(), pqpR2, pqpT2, model2.get(), PQP_FIRST_CONTACT);
    if (cres.NumPairs() > 0)
        return true;
    else
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
*  \date       2016-07-14
*/
template <unsigned int dim>
bool CollisionDetection<dim>::checkFCL(std::shared_ptr<FCLModel> &model1, std::shared_ptr<FCLModel> &model2, Matrix3 &R1,
                                       Matrix3 &R2, Vector3 &t1, Vector3 &t2) {
    o1 = new fcl::CollisionObject<float>(model1, R1, t1);
    o2 = new fcl::CollisionObject<float>(model2, R2, t2);
    fcl::CollisionRequest<float> request;    // default setting
    fcl::CollisionResult<float> result;
    collide(o1, o2, request, result);

    return result.isCollision();
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
