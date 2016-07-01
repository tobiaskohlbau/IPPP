#include <robot/RobotBase.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
RobotBase::RobotBase(std::string name, RobotType type, unsigned int dim, unsigned int numberJoints)
    : Base(name) {
    m_robotName = name;
    m_robotType = type;
    m_dim = dim;
    m_nbJoints = numberJoints;
}

/*!
*  \brief      Return dimension from the robot
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-06-30
*/
unsigned int RobotBase::getDim() const {
    return m_dim;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
unsigned int RobotBase::getNbJoints() const {
    return m_nbJoints;
}

/*!
*  \brief      Return name from the robot
*  \author     Sascha Kaden
*  \param[out] name
*  \date       2016-06-30
*/
std::string RobotBase::getName() const {
    return m_robotName;
}

/*!
*  \brief      Return the type of the robot
*  \author     Sascha Kaden
*  \param[out] robot type
*  \date       2016-06-30
*/
RobotType RobotBase::getType() const {
    return m_robotType;
}
