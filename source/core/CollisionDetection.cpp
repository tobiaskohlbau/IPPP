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

#include <core/CollisionDetection.h>

#include <core/Logging.h>
#include <core/Utilities.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
CollisionDetection::CollisionDetection(const shared_ptr<RobotBase> &robot) : Base("CollisionDetection") {
    m_robot = robot;

    if (m_robot->getCollisionType() == CollisionType::twoD)
        m_2DWorkspace = m_robot->get2DWorkspace();
    if (m_robot->getCollisionType() != CollisionType::twoD)
        m_workspace = m_robot->getWorkspace();
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vec
*  \param[out] binary result of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const Vec<float> &vec) {
    assert(vec.getDim() == m_robot->getDim());

    switch (m_robot->getCollisionType()) {
        case CollisionType::twoD:
            return controlCollisionPointRobot(vec[0], vec[1]);
        case CollisionType::pqp:
            return controlCollisionPqp(vec);
        case CollisionType::fcl:
            return controlCollisionFcl(vec);
    }
    return false;
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vector of points
*  \param[out] binary result of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const std::vector<Vec<float>> &vecs) {
    if (vecs.size() == 0)
        return false;

    assert(vecs[0].getDim() == m_robot->getDim());

    switch (m_robot->getCollisionType()) {
        case CollisionType::twoD:
            for (int i = 0; i < vecs.size(); ++i)
                if (controlCollisionPointRobot(vecs[i][0], vecs[i][1]))
                    return true;
            break;
        case CollisionType::pqp:
            for (int i = 0; i < vecs.size(); ++i)
                if (controlCollisionPqp(vecs[i]))
                    return true;
            break;
        case CollisionType::fcl:
            for (int i = 0; i < vecs.size(); ++i)
                if (controlCollisionFcl(vecs[i]))
                    return true;
            break;
    }
    return false;
}

/*!
*  \brief      Check for 2D PointRobot collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] binary result of collision
*  \date       2016-06-30
*/
bool CollisionDetection::controlCollisionPointRobot(float x, float y) {
    if (m_2DWorkspace.rows() == -1) {
        Logging::warning("Empty workspace!", this);
        return false;
    }

    if (m_2DWorkspace(x, y) < 80) {
        return true;
    } else {
        return false;
    }
}

/*!
*  \brief      Control collision with the PQP library
*  \author     Sascha Kaden
*  \param[in]  Vec of angles
*  \param[out] binary result of collision
*  \date       2016-07-14
*/
bool CollisionDetection::controlCollisionPqp(const Vec<float> &vec) {
    if (m_robot->getRobotType() == RobotType::mobile)
        return checkPqpMobileRobot(vec);
    else
        return checkPqpSerialRobot(vec);
}

bool CollisionDetection::checkPqpSerialRobot(const Vec<float> &vec) {
    shared_ptr<SerialRobot> robot(std::static_pointer_cast<SerialRobot>(m_robot));
    shared_ptr<PQP_Model> baseMesh = robot->getBase()->getPqp();
    std::vector<shared_ptr<PQP_Model>> models = robot->getJointPqpModels();

    std::vector<Eigen::Matrix4f> jointTrafos = robot->getJointTrafos(vec);
    Eigen::Matrix4f pose = robot->getPoseMat();
    std::vector<Eigen::Matrix4f> As;
    As.push_back(pose * jointTrafos[0]);
    for (int i = 1; i < jointTrafos.size(); ++i)
        As.push_back(As[i - 1] * jointTrafos[i]);

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t1, t2;
    Utilities::decomposeT(pose, R1, t1);
    // control collision of the robot base with the joints
    for (int i = 1; i < robot->getNbJoints(); ++i) {
        // get R and t from A for first model
        Utilities::decomposeT(As[i], R2, t2);
        if (checkPQP(baseMesh, models[i], R1, R2, t1, t2))
            return true;
    }

    // control collision of the robot joints with themself
    for (int i = 0; i < robot->getNbJoints(); ++i) {
        // get R and t from A for first model
        Utilities::decomposeT(As[i], R1, t1);

        for (int j = i + 2; j < robot->getNbJoints(); ++j) {
            // get R and t from A for second model
            Utilities::decomposeT(As[j], R2, t2);
            if (checkPQP(models[i], models[j], R1, R2, t1, t2)) {
#ifdef DEBUG_OUTPUT
                Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                Eigen::Vector3f r = As[i].block<3, 3>(0, 0).eulerAngles(0, 1, 2);
                Eigen::Vector3f t = As[i].block<3, 1>(0, 3);
                std::cout << "A" << i << ": ";
                std::cout << "Euler angles: " << r.transpose() << "\t";
                std::cout << "Translation: " << t.transpose() << std::endl;
                r = As[j].block<3, 3>(0, 0).eulerAngles(0, 1, 2);
                t = As[j].block<3, 1>(0, 3);
                std::cout << "A" << j << ": ";
                std::cout << "Euler angles: " << r.transpose() << "\t";
                std::cout << "Translation: " << t.transpose() << std::endl << std::endl;
#endif
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_robot->getWorkspace() != nullptr) {
        shared_ptr<PQP_Model> workspace = m_robot->getWorkspace()->getPqp();
        R2 = Eigen::Matrix3f::Zero(3, 3);
        for (int i = 0; i < 3; ++i) {
            R2(i, i) = 1;
            t2(i) = 0;
        }

        for (int i = 0; i < robot->getNbJoints(); ++i) {
            // get R and t from A for first model
            Utilities::decomposeT(As[i], R1, t1);
            if (checkPQP(models[i], workspace, R1, R2, t1, t2)) {
                Logging::debug("Collision between workspace and link " + std::to_string(i), this);
                return true;
            }
        }
    }

    return false;
}

bool CollisionDetection::checkPqpMobileRobot(const Vec<float> &vec) {
    return false;
}

bool CollisionDetection::controlCollisionFcl(const Vec<float> &vec) {
    if (m_robot->getRobotType() == RobotType::mobile)
        return checkPqpMobileRobot(vec);
    else
        return checkPqpSerialRobot(vec);
}

bool CollisionDetection::checkFclMobileRobot(const Vec<float> &vec) {
    return false;
}

bool CollisionDetection::checkFclSerialRobot(const Vec<float> &vec) {
    shared_ptr<SerialRobot> robot(std::static_pointer_cast<SerialRobot>(m_robot));
    shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> baseMesh = robot->getBase()->getFcl();
    std::vector<shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>>> models = robot->getJointFclModels();

    std::vector<Eigen::Matrix4f> jointTrafos = robot->getJointTrafos(vec);
    Eigen::Matrix4f pose = robot->getPoseMat();
    std::vector<Eigen::Matrix4f> As;
    As.push_back(pose * jointTrafos[0]);
    for (int i = 1; i < jointTrafos.size(); ++i)
        As.push_back(As[i - 1] * jointTrafos[i]);

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t1, t2;
    Utilities::decomposeT(pose, R1, t1);
    // control collision of the robot base with the joints
    for (int i = 1; i < As.size(); ++i) {
        // get R and t from A for first model
        Utilities::decomposeT(As[i], R2, t2);
        if (checkFcl(baseMesh, models[i], R1, R2, t1, t2))
            return true;
    }

    // control collision of the robot joints with themself
    for (int i = 0; i < As.size(); ++i) {
        // get R and t from A for first model
        Utilities::decomposeT(As[i], R1, t1);

        for (int j = i + 2; j < As.size(); ++j) {
            // get R and t from A for second model
            Utilities::decomposeT(As[j], R2, t2);
            if (checkFcl(models[i], models[j], R1, R2, t1, t2)) {
#ifdef DEBUG_OUTPUT
                Logging::debug("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), this);
                Eigen::Vector3f r = As[i].block<3, 3>(0, 0).eulerAngles(0, 1, 2);
                Eigen::Vector3f t = As[i].block<3, 1>(0, 3);
                std::cout << "A" << i << ": ";
                std::cout << "Euler angles: " << r.transpose() << "\t";
                std::cout << "Translation: " << t.transpose() << std::endl;
                r = As[j].block<3, 3>(0, 0).eulerAngles(0, 1, 2);
                t = As[j].block<3, 1>(0, 3);
                std::cout << "A" << j << ": ";
                std::cout << "Euler angles: " << r.transpose() << "\t";
                std::cout << "Translation: " << t.transpose() << std::endl << std::endl;
#endif
                return true;
            }
        }
    }

    // control collision with workspace
    if (m_robot->getWorkspace() != nullptr) {
        shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> workspace = m_robot->getWorkspace()->getFcl();
        R2 = Eigen::Matrix3f::Zero(3, 3);
        for (int i = 0; i < 3; ++i) {
            R2(i, i) = 1;
            t2(i) = 0;
        }

        for (int i = 0; i < As.size(); ++i) {
            // get R and t from A for first model
            Utilities::decomposeT(As[i], R1, t1);
            if (checkFcl(models[i], workspace, R1, R2, t1, t2)) {
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
*  \param[in]  PQP_Model one
*  \param[in]  PQP_Model two
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
    PQP_Model *m1 = model1.get();
    PQP_Model *m2 = model2.get();
    PQP_Collide(&cres, pqpR1, pqpT1, m1, pqpR2, pqpT2, m2, PQP_FIRST_CONTACT);
    if (cres.NumPairs() > 0)
        return true;
    else
        return false;
}

/*!
*  \brief      Check for collision with Fcl library
*  \author     Sascha Kaden
*  \param[in]  fcl model one
*  \param[in]  fcl model two
*  \param[in]  rotation matrix one
*  \param[in]  rotation matrix two
*  \param[in]  translation vector one
*  \param[in]  translation vector two
*  \param[out] binary result of collision
*  \date       2016-07-14
*/
bool CollisionDetection::checkFcl(shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> &model1,
                                  shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> &model2, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                                  Eigen::Vector3f &t1, Eigen::Vector3f &t2) {
    fcl::CollisionObject<float> *o1 = new fcl::CollisionObject<float>(model1, R1, t1);
    fcl::CollisionObject<float> *o2 = new fcl::CollisionObject<float>(model2, R2, t2);
    fcl::CollisionRequest<float> request; // default setting
    fcl::CollisionResult<float> result;
    collide(o1, o2, request, result);

    delete o1;
    delete o2;

    return result.isCollision();
}
