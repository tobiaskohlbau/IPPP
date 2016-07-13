#include <robot/Jaco.h>

using namespace rmpl;

#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
 #endif

/*!
*  \brief      Constructor of the Jaco robot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
Jaco::Jaco()
    : RobotBase("Jaco", RobotType::JACO, 6, 6) {
    this->m_alpha = Vec<float>(this->m_pi/2, this->m_pi, this->m_pi/2, 0.95993, 0.95993, this->m_pi);
    this->m_a = Vec<float>(0, 410, 0, 0, 0, 0);
    this->m_d = Vec<float>(275.5, 0, -9.8, -249.18224, -83.76448, -210.58224);

    char cCurrentPath[FILENAME_MAX];
    if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath))) {
        this->sendMessage("could not load cad files, wrong path");
        return;
    }
    cCurrentPath[sizeof(cCurrentPath) - 1] = '\0'; /* not really required */
    std::string path(cCurrentPath);

    this->m_cadFiles = {"/home/sascha/projects/Planner/meshes/link_base_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_1_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_2_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_3_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_4_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_5_fixed_origin.obj",
                        "/home/sascha/projects/Planner/meshes/link_hand_fixed_origin.obj"};

    // load cad models
    for ( auto file : m_cadFiles)
        m_cadModels.push_back(this->m_fileLoader->loadFile(file));
}

Vec<float> Jaco::directKinematic(const Vec<float> &angles) {
    std::vector<Eigen::Matrix4f> trafos = getTransformations(angles);

    Vec<float> basis(0,0,0,0,0,0);
    return getTcpPosition(trafos, basis);
}

std::vector<Eigen::Matrix4f> Jaco::getTransformations(const Vec<float> &angles) {
    // transform form jaco physical angles to dh angles
    Vec<float> dhAngles(angles);
    dhAngles[0] = -angles[0];
    dhAngles[1] = angles[1] - 90;
    dhAngles[2] = angles[2] + 90;
    dhAngles[4] = angles[4] - 180;
    dhAngles[5] = angles[5] + 100;

    Vec<float> rads = this->degToRad(dhAngles);

    std::vector<Eigen::Matrix4f> trafos;
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero(4,4);
    for (int i = 0; i < 4; ++i)
        A(i,i) = 1;
    trafos.push_back(A);

    // create transformation matrizes
    for (int i = 0; i < 6; ++i) {
        A = this->getTrafo(this->m_alpha[i], this->m_a[i], m_d[i], rads[i]);
        trafos.push_back(A);
    }
    return trafos;
}
