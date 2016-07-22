#include <robot/PointRobot.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the 2D PointRobot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
PointRobot::PointRobot()
    : RobotBase("PointRobot", CollisionType::twoD, 2, 0) {

}

Vec<float> PointRobot::directKinematic(const Vec<float> &angles) {
    return angles;
}

std::vector<Eigen::Matrix4f> PointRobot::getTransformations(const Vec<float> &angles) {
    return std::vector<Eigen::Matrix4f>();
}
