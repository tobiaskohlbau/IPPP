#include <robot/PointRobot.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the 2D PointRobot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
PointRobot::PointRobot()
    : RobotBase("PointRobot", RobotType::POINT_ROBOT, 2, 0) {

}

Vec<float> PointRobot::directKinematic(const Vec<float> &angles) {
    return angles;
}
