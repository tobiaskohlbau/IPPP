//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//-------------------------------------------------------------------------//

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
    : RobotBase("Jaco", CollisionType::pqp, 6, 6) {
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

    std::vector<std::string> cadFiles = {"/home/sascha/projects/Planner/meshes/link_base_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_1_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_2_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_3_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_4_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_5_fixed_origin.obj",
                            "/home/sascha/projects/Planner/meshes/link_hand_fixed_origin.obj"};

    // load cad models
    this->setCadModels(cadFiles);
}

Vec<float> Jaco::directKinematic(const Vec<float> &angles) {
    std::vector<Eigen::Matrix4f> trafos = getTransformations(angles);

    Vec<float> basis(0,0,0,0,0,0);
    return getTcpPosition(trafos, basis);
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-07-14
*/
std::vector<Eigen::Matrix4f> Jaco::getTransformations(const Vec<float> &angles) {
    // transform form jaco physical angles to dh angles
    Vec<float> dhAngles = convertRealToDH(angles);
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

/*!
*  \brief      Convert real angles to D-H Angles
*  \detail     This conversation is a kinova jaco own issue
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] D-H angles
*  \date       2016-07-14
*/
Vec<float> Jaco::convertRealToDH(const Vec<float> &realAngles) {
    Vec<float> dhAngles(realAngles);
    dhAngles[0] = -realAngles[0];
    dhAngles[1] = realAngles[1] - 90;
    dhAngles[2] = realAngles[2] + 90;
    dhAngles[4] = realAngles[4] - 180;
    dhAngles[5] = realAngles[5] + 100;

    return dhAngles;
}
