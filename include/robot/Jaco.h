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

private:
    Vec<float> m_alpha;
    Vec<float> m_a;
    Vec<float> m_d;
};

} /* namespace rmpl */

#endif /* JACO_H_ */
