#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <Eigen/Core>

#include <core/Node.h>
#include <core/Base.h>
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
    bool controlCollision(const std::shared_ptr<Node> &node);
    bool controlCollision(const Vec<float> &vec);
    bool controlCollision(const std::vector<Vec<float>> &vec);
    bool controlCollisionPointRobot(const float &x, const float &y);
    bool controlCollisionVrep(const Vec<float> &vec);
    bool controlCollisionVrep(const std::vector<Vec<float>> &vec);

    void set2DWorkspace(Eigen::MatrixXi space);
    Eigen::MatrixXi get2DWorkspace() const;
private:
    std::shared_ptr<RobotBase> m_robot;
    std::shared_ptr<Helper>    m_vrep;
    Eigen::MatrixXi m_workspace;
};

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
