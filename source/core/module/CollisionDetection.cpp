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

#include <core/module/CollisionDetection.h>

#include <core/utility/Logging.h>
#include <core/utility/Utility.h>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
CollisionDetection::CollisionDetection(const shared_ptr<RobotBase> &robot) : ModuleBase("CollisionDetection") {
    m_robot = robot;
    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();

    m_identity = Eigen::Matrix3f::Identity(3, 3);
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
bool CollisionDetection::controlVec(const Eigen::VectorXf &vec) {
    assert(vec.rows() == m_robot->getDim());

    if (m_robot->getCollisionType() == CollisionType::point2D)
        return checkPoint2D(vec[0], vec[1]);
    else if (m_robot->getCollisionType() == CollisionType::triangle2D)
        return checkTriangleRobot(vec);
    else
        return controlCollisionMesh(vec);
}

bool CollisionDetection::controlCollisionMesh(const Eigen::VectorXf &vec) {
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
bool CollisionDetection::controlTrajectory(std::vector<Eigen::VectorXf> &vecs) {
    if (vecs.size() == 0)
        return false;

    assert(vecs[0].rows() == m_robot->getDim());

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
bool CollisionDetection::checkPoint2D(float x, float y) {
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
*  \param[in]  transformation
*  \param[out] binary result of collision
*  \date       2016-06-30
*/
bool CollisionDetection::checkTriangleRobot(const Eigen::Vector3f &vec) {
    shared_ptr<TriangleRobot2D> robot(std::static_pointer_cast<TriangleRobot2D>(m_robot));
    std::vector<Triangle2D> triangles = robot->getTriangles();

    Eigen::Matrix2f R;
    Eigen::Vector2f t;
    utility::poseVecToRandT(vec, R, t);

    Eigen::Vector2f u, temp;
    for (auto triangle : triangles) {
        triangle.transform(R, t);

        u = triangle.getP(2) - triangle.getP(1);
        u /= u.norm();
        temp = triangle.getP(1);
        while ((temp - triangle.getP(2)).squaredNorm() > 2) {
            if (checkPoint2D(temp[0], temp[1])) {
                Logging::debug("Collision of triangle", this);
                return true;
            }
            temp += u;
        }

        u = triangle.getP(3) - triangle.getP(1);
        u /= u.norm();
        temp = triangle.getP(1);
        while ((temp - triangle.getP(3)).squaredNorm() > 2) {
            if (checkPoint2D(temp[0], temp[1])) {
                Logging::debug("Collision of triangle", this);
                return true;
            }
            temp += u;
        }

        u = triangle.getP(3) - triangle.getP(2);
        u /= u.norm();
        temp = triangle.getP(2);
        while ((temp - triangle.getP(3)).squaredNorm() > 2) {
            if (checkPoint2D(temp[0], temp[1])) {
                Logging::debug("Collision of triangle", this);
                return true;
            }
            temp += u;
        }
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
bool CollisionDetection::checkSerialRobot(const Eigen::VectorXf &vec) {
    shared_ptr<SerialRobot> robot(std::static_pointer_cast<SerialRobot>(m_robot));

    std::vector<Eigen::Matrix4f> jointTrafos = robot->getJointTrafos(vec);
    Eigen::Matrix4f pose = robot->getPoseMat();
    Eigen::Matrix4f As[jointTrafos.size()];
    As[0] = pose * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i)
        As[i] = As[i - 1] * jointTrafos[i];

    Eigen::Matrix3f poseR;
    Eigen::Vector3f poseT;
    utility::decomposeT(pose, poseR, poseT);

    Eigen::Matrix3f rot[jointTrafos.size()];
    Eigen::Vector3f trans[jointTrafos.size()];
    for (int i = 0; i < jointTrafos.size(); ++i)
        utility::decomposeT(As[i], rot[i], trans[i]);

    if (m_robot->getCollisionType() == CollisionType::pqp) {
        shared_ptr<PQP_Model> baseMesh = nullptr;
        if (robot->getBaseMesh() != nullptr)
            baseMesh = robot->getBaseMesh()->m_pqpModel;
        std::vector<shared_ptr<PQP_Model>> jointMeshes = robot->getJointPqpModels();

        //        for (auto tmp : jointTrafos)
        //            std::cout << tmp <<std::endl;
        //        robot->saveMeshConfig(As);

        return checkMesh(jointMeshes, baseMesh, rot, poseR, trans, poseT);
    } else {
        shared_ptr<FCLModel> baseMesh = nullptr;
        if (robot->getBaseMesh() != nullptr)
            baseMesh = robot->getBaseMesh()->m_fclModel;
        std::vector<shared_ptr<FCLModel>> jointMeshes = robot->getJointFclModels();
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
bool CollisionDetection::checkMobileRobot(const Eigen::VectorXf &vec) {
    Eigen::Matrix4f pose = m_robot->getPoseMat();
    Eigen::Matrix3f poseR;
    Eigen::Vector3f poseT;
    utility::decomposeT(pose, poseR, poseT);

    if (m_robot->getCollisionType() == CollisionType::pqp) {
        if (m_robot->getBaseMesh() == nullptr || m_robot->getWorkspace() == nullptr)
            return false;

        shared_ptr<PQP_Model> baseMesh = m_robot->getBaseMesh()->m_pqpModel;
        shared_ptr<PQP_Model> workspace = m_robot->getWorkspace()->getPqp();

        return checkPQP(workspace, baseMesh, m_identity, poseR, m_zeroVec, poseT);
    } else {
        if (m_robot->getBaseMesh() == nullptr || m_robot->getWorkspace() == nullptr)
            return false;

        shared_ptr<FCLModel> baseMesh = m_robot->getBaseMesh()->m_fclModel;
        shared_ptr<FCLModel> workspace = m_robot->getWorkspace()->getFcl();

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
bool CollisionDetection::checkMesh(std::vector<shared_ptr<PQP_Model>> &models, shared_ptr<PQP_Model> &baseModel,
                                   Eigen::Matrix3f R[], Eigen::Matrix3f &poseR, Eigen::Vector3f t[], Eigen::Vector3f &poseT) {
    // control collision between baseModel and joints
    if (baseModel != nullptr) {
        for (int i = 1; i < m_robot->getDim(); ++i) {
            if (checkPQP(baseModel, models[i], poseR, R[i], poseT, t[i]))
                return true;
        }
    }

    // control collision of the robot joints with themselves
    for (int i = 0; i < m_robot->getDim(); ++i) {
        for (int j = i + 2; j < m_robot->getDim(); ++j) {
            if (checkPQP(models[i], models[j], R[i], R[j], t[i], t[j])) {
                if (Logging::getLogLevel() == LogLevel::debug) {
                    shared_ptr<SerialRobot> robot(std::static_pointer_cast<SerialRobot>(m_robot));
                    Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                    Eigen::Vector3f r = R[i].eulerAngles(0, 1, 2);
                    std::cout << "A" << i << ": ";
                    std::cout << "Euler angles: " << std::endl << R[i] << std::endl;
                    std::cout << "Translation: " << t[i].transpose() << std::endl;
                    robot->getMeshFromJoint(i)->saveObj(std::to_string(i) + ".obj", utility::createT(R[i], t[i]));
                    r = R[j].eulerAngles(0, 1, 2);
                    std::cout << "A" << j << ": ";
                    std::cout << "Euler angles: " << std::endl << R[j] << std::endl;
                    std::cout << "Translation: " << t[j].transpose() << std::endl << std::endl;
                    robot->getMeshFromJoint(j)->saveObj(std::to_string(j) + ".obj", utility::createT(R[i], t[i]));
                }
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_robot->getWorkspace() != nullptr) {
        shared_ptr<PQP_Model> workspace = m_robot->getWorkspace()->getPqp();

        if (checkPQP(workspace, baseModel, m_identity, poseR, m_zeroVec, poseT))
            return true;

        for (int i = 0; i < m_robot->getDim(); ++i) {
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
bool CollisionDetection::checkMesh(std::vector<shared_ptr<FCLModel>> &models, shared_ptr<FCLModel> &baseModel,
                                   Eigen::Matrix3f R[], Eigen::Matrix3f &poseR, Eigen::Vector3f t[], Eigen::Vector3f &poseT) {
    // control collision between baseModel and joints
    if (baseModel != nullptr) {
        for (int i = 1; i < m_robot->getDim(); ++i) {
            if (checkFCL(baseModel, models[i], poseR, R[i], poseT, t[i]))
                return true;
        }
    }

    // control collision of the robot joints with themself
    for (int i = 0; i < m_robot->getDim(); ++i) {
        for (int j = i + 2; j < m_robot->getDim(); ++j) {
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
        shared_ptr<FCLModel> workspace = m_robot->getWorkspace()->getFcl();

        if (checkFCL(workspace, baseModel, m_identity, poseR, m_zeroVec, poseT))
            return true;

        for (int i = 0; i < m_robot->getDim(); ++i) {
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
bool CollisionDetection::checkPQP(shared_ptr<PQP_Model> &model1, shared_ptr<PQP_Model> &model2, Eigen::Matrix3f &R1,
                                  Eigen::Matrix3f &R2, Eigen::Vector3f &t1, Eigen::Vector3f &t2) {
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
bool CollisionDetection::checkFCL(shared_ptr<FCLModel> &model1, shared_ptr<FCLModel> &model2, Eigen::Matrix3f &R1,
                                  Eigen::Matrix3f &R2, Eigen::Vector3f &t1, Eigen::Vector3f &t2) {
    o1 = new fcl::CollisionObject<float>(model1, R1, t1);
    o2 = new fcl::CollisionObject<float>(model2, R2, t2);
    fcl::CollisionRequest<float> request;    // default setting
    fcl::CollisionResult<float> result;
    collide(o1, o2, request, result);

    return result.isCollision();
}

} /* namespace rmpl */