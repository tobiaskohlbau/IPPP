#include <robot/RobotBase.h>

using namespace rmpl;

RobotBase::RobotBase(std::string name, unsigned int dim, unsigned int numberJoints)
    : Base(name) {
    m_robotName = name;
    m_dim = dim;
    m_nbJoints = numberJoints;
}

unsigned int RobotBase::getDim() const {
    return m_dim;
}

unsigned int RobotBase::getNbJoints() const {
    return m_nbJoints;
}
