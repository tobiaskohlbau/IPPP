#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <Eigen/Dense>

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
    unsigned int getDim() const;
    unsigned int getNbJoints() const;
    std::string getName() const;
    RobotType getType() const;

private:
    std::string  m_robotName;
    RobotType    m_robotType;
    unsigned int m_nbJoints;
    unsigned int m_dim;
    Eigen::MatrixXi m_matrix;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
