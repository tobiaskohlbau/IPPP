#include <robot/Jaco.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the Jaco robot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
Jaco::Jaco()
    : RobotBase("Jaco", RobotType::JACO, 6, 6) {
    m_alpha = Vec<float>(this->m_pi/2, this->m_pi, this->m_pi/2, 0.95993, 0.95993, this->m_pi);
    m_a = Vec<float>(0, 0.40, 0, 0 ,0 ,0);
    m_d = Vec<float>(0.2755, 0, -0.0098, -0.2721, -0.1295, -0.233);
}

Vec<float> Jaco::directKinematic(const Vec<float> &angles) {
    Vec<float> rads = this->degToRad(angles);

    // create transformation matrizes
    std::vector<Eigen::Matrix4f> trafos;
    for (int i = 0; i < 6; ++i) {
        Eigen::Matrix4f A = this->getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]);
        trafos.push_back(A);
    }

    Vec<float> basis(0,0,0,0,0,0);
    return getTcpPosition(trafos, basis);
}
