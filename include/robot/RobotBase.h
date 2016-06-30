#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <Eigen/Dense>

#include <core/Base.h>
#include <core/Vec.hpp>

namespace rmpl {

/*!
* \brief   Base class robot
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Base
{
public:
    RobotBase(std::string name, unsigned int dim, unsigned int numberJoints);
    unsigned int getDim() const;
    unsigned int getNbJoints() const;

private:
    std::string m_robotName;
    unsigned int m_nbJoints;
    unsigned int m_dim;
    Eigen::MatrixXd m_matrix;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
