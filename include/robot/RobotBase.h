#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <Eigen/Core>

#include <core/Base.h>
#include <core/Vec.hpp>

namespace rmpl {

enum RobotType
{
    JACO,
    POINT_ROBOT
};

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Base
{
public:
    RobotBase(std::string name, RobotType type, unsigned int dim, unsigned int numberJoints);

    virtual Vec<float> directKinematic(const Vec<float> &angles) = 0;

    unsigned int getDim() const;
    unsigned int getNbJoints() const;
    std::string getName() const;
    RobotType getType() const;

    Eigen::Matrix4f getTrafo(const float &alpha, const float &a, const float &d, const float q);
    Vec<float> getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<float> basis);

    Vec<float> degToRad(const Vec<float> deg);
    Eigen::ArrayXf getEigenVec(const Vec<float> vec);

protected:
    std::string  m_robotName;
    RobotType    m_robotType;
    unsigned int m_nbJoints;
    unsigned int m_dim;
    float        m_pi = 3.1416;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
