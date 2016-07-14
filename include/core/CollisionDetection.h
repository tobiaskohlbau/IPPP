#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <Eigen/Core>
#include <PQP/include/PQP.h>

#include <core/Base.h>
#include <core/Node.h>
#include <robot/RobotBase.h>
#include <vrep/Helper.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return boolean value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetection : public Base
{
public:
    CollisionDetection(const std::shared_ptr<Helper> &vrep, const std::shared_ptr<RobotBase> &robot);
    bool controlCollision(const Vec<float> &vec);
    bool controlCollision(const std::vector<Vec<float>> &vec);

private:
    bool controlCollisionPQP(const Vec<float> &vec);
    bool checkPQP(std::shared_ptr<PQP_Model> model1, std::shared_ptr<PQP_Model> model2, Eigen::Matrix3f R1, Eigen::Matrix3f R2, Eigen::Vector3f t1, Eigen::Vector3f t2);
    bool controlCollisionPointRobot(float x, float y);
    bool controlCollisionVrep(const Vec<float> &vec);
    bool controlCollisionVrep(const std::vector<Vec<float>> &vec);

    std::shared_ptr<RobotBase> m_robot;
    std::shared_ptr<Helper>    m_vrep;
    Eigen::MatrixXi m_2DWorkspace;
};

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
