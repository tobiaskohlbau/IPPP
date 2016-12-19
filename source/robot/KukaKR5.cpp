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

#include <robot/KukaKR5.h>

#include <include/core/utility/Utility.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date        2016-10-22
*/
KukaKR5::KukaKR5() : SerialRobot("KukaKR5", CollisionType::pqp, 6) {
    m_alpha = Vec<float>(90, 0, 90, 90, 90, 0);
    m_alpha = utility::degToRad(m_alpha);
    m_a = Vec<float>(180, 600, 120, 0, 0, 0);
    m_d = Vec<float>(400, 0, 0, 620, 0, 115);
    m_pose = Vec<float>(0, 0, 0, 0, 0, 0);

    m_baseMesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link0.stl"));

    std::shared_ptr<MeshContainer> mesh(new MeshContainer("meshes/KukaKR5/link1.stl"));
    Joint joint(-155, 155, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link2.stl"));
    joint = Joint(-65, 180, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link3.stl"));
    joint = Joint(-68, 105, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link4.stl"));
    joint = Joint(-350, 350, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link5.stl"));
    joint = Joint(50, 310, mesh);
    m_joints.push_back(joint);
    mesh = std::shared_ptr<MeshContainer>(new MeshContainer("meshes/KukaKR5/link6.stl"));
    joint = Joint(-530, 170, mesh);
    m_joints.push_back(joint);

    m_minBoundary = Vec<float>(-155, -65, -68, -350, 50, -530);
    m_maxBoundary = Vec<float>(155, 180, 105, 350, 310, 170);
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-10-22
*/
Vec<float> KukaKR5::directKinematic(const Vec<float> &angles) {
    std::vector<Eigen::Matrix4f> trafos = getJointTrafos(angles);

    return getTcpPosition(trafos);
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-10-22
*/
std::vector<Eigen::Matrix4f> KukaKR5::getJointTrafos(const Vec<float> &angles) {
    Vec<float> rads = utility::degToRad(angles);

    std::vector<Eigen::Matrix4f> trafos;
    for (int i = 0; i < getDim(); ++i)
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]));
    return trafos;
}