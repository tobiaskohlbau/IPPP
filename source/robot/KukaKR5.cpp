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

#include <core/utility/Utility.h>

namespace rmpl {

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date        2016-10-22
*/
KukaKR5::KukaKR5()
    : SerialRobot<6>("KukaKR5", CollisionType::pqp, utilVec::Vecf(-155, -65, -68, -350, 50, -530),
                     utilVec::Vecf(155, 180, 105, 350, 310, 170)) {
    m_alpha = utilVec::Vecf(90, 0, 90, 90, 90, 0);
    m_alpha = utilGeo::degToRad<6>(m_alpha);
    m_a = utilVec::Vecf(180, 600, 120, 0, 0, 0);
    m_d = utilVec::Vecf(400, 0, 0, 620, 0, 115);
    m_pose = utilVec::Vecf(0, 0, 0, 0, 0, 0);

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
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-10-22
*/
Vector6 KukaKR5::directKinematic(const Vector6 &angles) {
    std::vector<Matrix4> trafos = getJointTrafos(angles);

    return getTcpPosition(trafos);
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-10-22
*/
std::vector<Matrix4> KukaKR5::getJointTrafos(const Vector6 &angles) {
    Vector6 rads = utilGeo::degToRad<6>(angles);

    std::vector<Matrix4> trafos;
    for (int i = 0; i < 6; ++i)
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]));
    return trafos;
}

} /* namespace rmpl */