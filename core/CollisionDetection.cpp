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

    switch (m_robot->getType()) {
        case RobotType::POINT_ROBOT:
            return controlCollisionPointRobot(vec[0], vec[1]);
            break;
        case RobotType::JACO:
            return controlCollisionPQP(vec);
            break;
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

    switch (m_robot->getType()) {
        case RobotType::POINT_ROBOT:
            for (int i = 0; i < vecs.size(); ++i)
                if (controlCollisionPointRobot(vecs[i][0], vecs[i][1]))
                    return true;
            break;
        case RobotType::JACO:
            for (int i = 0; i < vecs.size(); ++i)
                if (controlCollisionPQP(vecs[i]))
                    return true;
            break;
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
bool CollisionDetection::controlCollisionPointRobot(const float &x, const float &y) {
    if (m_2Dworkspace.rows() == -1 && m_2Dworkspace.cols() == -1) {
        this->sendMessage("Empty workspace!");
        return false;
    }

    if (m_2Dworkspace(x,y) < 30) {
        return true;
    }
    else {
        return false;
    }
    return false;
}

bool CollisionDetection::controlCollisionPQP(const Vec<float> &vec) {
    std::vector<Eigen::Matrix4f> trafos;
    trafos = m_robot->getTransformations(vec);
    std::vector<Eigen::Matrix4f> As;
    As.push_back(trafos[0]);
    for (int i = 1; i < trafos.size(); ++i)
        As.push_back(As[i-1] * trafos[i]);

    Eigen::Matrix3f R1, R2;
    Eigen::Vector3f t1, t2;
    for (int i = 0; i < As.size(); ++i) {
        // get R and t from A for first model
        R1 = As[i].block<3,3>(0,0);
        t1 = As[i].block<3,1>(0,3);
        for (int j = i + 2; j < As.size(); ++j) {
            // get R and t from A for second model
            R2 = As[j].block<3,3>(0,0);
            t2 = As[j].block<3,1>(0,3);
            if (checkPQP(m_robot->getCadModel(i), m_robot->getCadModel(j), R1, R2, t1, t2)) {
                this->sendMessage("Collision between link " + std::to_string(i) + " and link " + std::to_string(j));
                for (int k = 0; k < As.size(); ++k) {
                    std::cout << "A" << k << ":" << std::endl;
                    Eigen::Vector3f r = As[k].block<3,3>(0,0).eulerAngles(0, 1, 2);
                    std::cout << "Euler angles: " << r.transpose() << std::endl;
                    Eigen::Vector3f t = As[k].block<3,1>(0,3);
                    std::cout << "translation: " << t.transpose() << std::endl << std::endl;
                }
                return true;
            }
        }
    }
    return false;
}

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
*  \param[in]  vec
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

/*!
*  \brief      Set workspace for 2D collision check
*  \author     Sascha Kaden
*  \param[in]  workspace
*  \date       2016-05-25
*/
void CollisionDetection::set2DWorkspace(Eigen::MatrixXi space) {
    if (this->m_2Dworkspace.rows() == -1 || this->m_2Dworkspace.cols() == -1) {
        this->sendMessage("Empty space given!");
        return;
    }

    m_2Dworkspace = space;
}

/*!
*  \brief      Return workspace for 2D collision
*  \author     Sascha Kaden
*  \param[out] workspace
*  \date       2016-05-25
*/
Eigen::MatrixXi CollisionDetection::get2DWorkspace() const {
    return m_2Dworkspace;
}
