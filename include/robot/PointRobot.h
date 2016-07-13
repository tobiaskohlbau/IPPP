#ifndef POINTROBOT_H_
#define POINTROBOT_H_

#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Class for the 2D point robot
* \author  Sascha Kaden
* \date    2016-06-30
*/
class PointRobot : public RobotBase
{
public:
    PointRobot();
    Vec<float> directKinematic(const Vec<float> &angles);
    std::vector<Eigen::Matrix4f> getTransformations(const Vec<float> &angles);

private:
};

} /* namespace rmpl */

#endif /* POINTROBOT_H_ */
