#include <robot/RobotBase.h>

#include <math.h>

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

Eigen::Matrix4f RobotBase::getTrafo(const float &alpha, const float &a, const float &d, const float q) {
    float sinAlpha = sin(alpha);
    float cosAlpha = cos(alpha);
    float sinQ = sin(q);
    float cosQ = cos(q);

    Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);
    T(0,0) = cosQ;
    T(0,1) = -sinQ * cosAlpha;
    T(0,2) = sinQ * cosAlpha;
    T(0,3) = a * cosQ;
    T(1,0) = sinQ;
    T(1,1) = cosQ * cosAlpha;
    T(1,2) = -cosQ * sinAlpha;
    T(1,3) = a * sinQ;
    T(2,1) = sinAlpha;
    T(2,2) = cosAlpha;
    T(2,3) = d;
    T(3,3) = 1;
    return T;
}

Vec<float> RobotBase::getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<float> basis) {
    Eigen::MatrixXf basisEigen = getEigenVec(basis);

    // multiply these matrizes together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Eigen::Matrix4f T;
    T = trafos[0];
    for (int i = 1; i < 6; ++i)
        T *= trafos[i];

    Eigen::MatrixXf tcp = basisEigen * T;
}

Vec<float> RobotBase::degToRad(const Vec<float> deg) {
    Vec<float> rad(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        rad[i] = deg[i] / 360 * m_pi;
    return rad;
}


Eigen::ArrayXf RobotBase::getEigenVec(const Vec<float> vec) {
    Eigen::ArrayXf eigenVec(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        eigenVec(i, 0) = vec[i];
    return eigenVec;
}
