#ifndef JACO_H_
#define JACO_H_

#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Class for the jaco robot
* \author  Sascha Kaden
* \date    2016-06-30
*/
class Jaco : public RobotBase
{
public:
    Jaco();
    Vec<float> directKinematic(const Vec<float> &angles);
    std::vector<Eigen::Matrix4f> getTransformations(const Vec<float> &angles);

private:
    Vec<float> convertRealToDH(const Vec<float> &realAngles);
};

} /* namespace rmpl */

#endif /* JACO_H_ */
