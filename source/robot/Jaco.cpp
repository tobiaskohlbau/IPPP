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

#include <core/Utilities.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
Jaco::Jaco() : SerialRobot("Jaco", CollisionType::pqp, 6) {
    m_alpha = Vec<float>(Utilities::pi() / 2, Utilities::pi(), Utilities::pi() / 2, 0.95993, 0.95993, Utilities::pi());
    m_a = Vec<float>(0, 410, 0, 0, 0, 0);
    m_d = Vec<float>(275.5f, 0, -9.8f, -249.18224f, -83.76448f, -210.58224f);

    m_pose = Vec<float>(0,0,0,0,0,0);
    m_baseMesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_base_fixed_origin.obj"));

    std::shared_ptr<MeshContainer> mesh(new MeshContainer("meshes/link_1_fixed_origin.obj"));
    Joint joint(0, 360, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_2_fixed_origin.obj"));
    joint = Joint(42, 318, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_3_fixed_origin.obj"));
    joint = Joint(17, 343, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_4_fixed_origin.obj"));
    joint = Joint(0, 360, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_5_fixed_origin.obj"));
    joint = Joint(0, 360, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/link_hand_fixed_origin.obj"));
    joint = Joint(0, 360, mesh);
    m_joints.push_back(joint);
    m_minBoundary = Vec<float>(0, 42, 17, 0, 0, 0);
    m_maxBoundary = Vec<float>(360, 318, 343, 360, 360, 360);
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-08-25
*/
Vec<float> Jaco::directKinematic(const Vec<float> &angles) {
    std::vector<Eigen::Matrix4f> trafos = getJointTrafos(angles);

    return getTcpPosition(trafos);
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-07-14
*/
std::vector<Eigen::Matrix4f> Jaco::getJointTrafos(const Vec<float> &angles) {
    // transform form jaco physical angles to dh angles
    Vec<float> dhAngles = convertRealToDH(angles);
    Vec<float> rads = Utilities::degToRad(dhAngles);

    std::vector<Eigen::Matrix4f> trafos;
    // create transformation matrizes
    for (int i = 0; i < 6; ++i)
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]));

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
