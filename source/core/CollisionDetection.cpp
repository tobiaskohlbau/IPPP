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

#include <Eigen/Geometry>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
CollisionDetection::CollisionDetection(const shared_ptr<Helper> &vrep, const shared_ptr<RobotBase> &robot)
        : Base("CollisionDetection") {
    m_robot = robot;
    m_vrep = vrep;

    if (m_robot->getCollisionType() == CollisionType::twoD)
        m_2DWorkspace = m_robot->get2DWorkspace();
    if (m_robot->getCollisionType() == CollisionType::pqp)
        m_pqpWorkspace = m_robot->getWorkspace();
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vec
*  \param[out] possibility of collision
*  \date       2016-05-25
*/
bool CollisionDetection::controlCollision(const Vec<float> &vec) {
    assert(vec.getDim() == m_robot->getDim());

    switch (m_robot->getCollisionType()) {
        case CollisionType::twoD:
            return controlCollisionPointRobot(vec[0], vec[1]);
        case CollisionType::pqp:
            return controlCollisionPQP(vec);
        case CollisionType::vrep:
            return controlCollisionVrep(vec);
        default:
            return false;
    }
}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  vector of points
*  \param[out] possibility of collision
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
                if (controlCollisionPQP(vecs[i]))
                    return true;
            break;
        case CollisionType::vrep:
            return controlCollisionVrep(vecs);
        default:
            return false;
    }
}

/*!
*  \brief      Check for 2D PointRobot collision
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[out] possibility of collision, true if in collision
*  \date       2016-06-30
*/
bool CollisionDetection::controlCollisionPointRobot(float x, float y) {
    if (m_2DWorkspace.rows() == -1) {
        this->sendMessage("Empty workspace!");
        return false;
    }

    if (m_2DWorkspace(x,y) < 30) {
        return true;
    }
    else {
        return false;
    }
    return false;
}

/*!
*  \brief      Control collision with the PQP library
*  \author     Sascha Kaden
*  \param[in]  Vec of angles
*  \param[out] possibility of collision
*  \date       2016-07-14
*/
bool CollisionDetection::controlCollisionPQP(const Vec<float> &vec) {
    std::vector<Eigen::Matrix4f> trafos;
    trafos = m_robot->getTransformations(vec);
    std::vector<Eigen::Matrix4f> As;
    As.push_back(trafos[0]);
    for (int i = 1; i < trafos.size(); ++i)
        As.push_back(As[i-1] * trafos[i]);

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t1, t2;
    // control collision of the robot with itself
    for (int i = 0; i < As.size(); ++i) {
        // get R and t from A for first model
        R1 = As[i].block<3,3>(0,0);
        t1 = As[i].block<3,1>(0,3);

        for (int j = i + 2; j < As.size(); ++j) {
            // get R and t from A for second model
            R2 = As[j].block<3,3>(0,0);
            t2 = As[j].block<3,1>(0,3);
            if (checkPQP(m_robot->getCadModel(i), m_robot->getCadModel(j), R1, R2, t1, t2)) {
                this->sendMessage("Collision between link " + std::to_string(i) + " and link " + std::to_string(j), Message::debug);
                //Eigen::Vector3f r = As[i].block<3,3>(0,0).eulerAngles(0, 1, 2);
                //Eigen::Vector3f t = As[i].block<3,1>(0,3);
                //std::cout << "A" << i << ": ";
                //std::cout << "Euler angles: " << r.transpose() << "   ";
                //std::cout << "Translation: " << t.transpose() << std::endl;
                //r = As[j].block<3,3>(0,0).eulerAngles(0, 1, 2);
                //t = As[j].block<3,1>(0,3);
                //std::cout << "A" << j << ": ";
                //std::cout << "Euler angles: " << r.transpose() << "   ";
                //std::cout << "Translation: " << t.transpose() << std::endl << std::endl;

                return true;
            }
        }
    }

    // control collision with workspace
    // shared_ptr<PQP_Model> workspace = m_robot->getWorkspace();
    if (m_pqpWorkspace != nullptr) {
        R2 = Eigen::Matrix3f::Zero(3,3);
        for (int i = 0; i < 3; ++i) {
            R2(i,i) = 1;
            t2(i) = 0;
        }

        for (int i = 0; i < As.size(); ++i) {
            // get R and t from A for first model
            R1 = As[i].block<3,3>(0,0);
            t1 = As[i].block<3,1>(0,3);
            if (checkPQP(m_robot->getCadModel(i), m_pqpWorkspace, R1, R2, t1, t2))
                return true;
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
*  \param[in]  translation vector one
*  \param[out] possibility of collision
*  \date       2016-07-14
*/
bool CollisionDetection::checkPQP(shared_ptr<PQP_Model> model1, shared_ptr<PQP_Model> model2, Eigen::Matrix3f R1, Eigen::Matrix3f R2, Eigen::Vector3f t1, Eigen::Vector3f t2) {
    PQP_REAL pqpR1[3][3], pqpR2[3][3], pqpT1[3], pqpT2[3];

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pqpR1[i][j] = R1(i,j);
            pqpR2[i][j] = R2(i,j);
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
*  \brief      Check for vrep collision
*  \author     Sascha Kaden
*  \param[in]  Vec of angles
*  \param[out] possibility of collision
*  \date       2016-06-30
*/
bool CollisionDetection::controlCollisionVrep(const Vec<float> &vec) {
    assert(vec.getDim() == 6);
    return m_vrep->checkCollision(vec);
}

/*!
*  \brief      Check for vrep collision
*  \author     Sascha Kaden
*  \param[in]  vector of vec
*  \param[out] possibility of collision
*  \date       2016-06-30
*/
bool CollisionDetection::controlCollisionVrep(const std::vector<Vec<float>> &vecs) {
    assert(vecs[0].getDim() == 6);
    return m_vrep->checkCollision(vecs);
}
