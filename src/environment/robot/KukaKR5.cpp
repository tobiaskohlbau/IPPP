//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#include <ippp/environment/robot/KukaKR5.h>

#include <ippp/environment/model/ModelFactoryPqp.h>

namespace ippp {

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date        2016-10-22
*/
KukaKR5::KukaKR5()
    : SerialRobot("KukaKR5", 6,
                  std::make_pair(util::Vecd(-2.70526, -1.13446, -1.18682, -6.10865, 0.872665, -util::twoPi()),
                                 util::Vecd(2.70526, util::pi(), 1.8326f, 6.10865f, 5.41052f, 2.96706f)),
                  std::vector<DofType>({DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                        DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot})) {
    m_alpha = util::Vecd(90, 0, 90, 90, 90, 0);
    m_alpha = util::degToRad<6>(m_alpha);
    m_a = util::Vecd(180, 600, 120, 0, 0, 0);
    m_d = util::Vecd(400, 0, 0, 620, 0, 115);
    m_pose = util::Vecd(0, 0, 0, 0, 0, 0);

    ModelFactoryPqp modelFactoryPqp;
    m_baseModel = modelFactoryPqp.createModel("meshes/KukaKR5/link0.stl");
    Joint joint;
    // -155, 155
    joint = Joint(-2.70526f, 2.70526f, modelFactoryPqp.createModel("meshes/KukaKR5/link1.stl"));
    m_joints.push_back(joint);
    // -65, 180
    joint = Joint(-1.13446f, util::pi(), modelFactoryPqp.createModel("meshes/KukaKR5/link2.stl"));
    m_joints.push_back(joint);
    // -68, 105
    joint = Joint(-1.18682f, 1.8326f, modelFactoryPqp.createModel("meshes/KukaKR5/link3.stl"));
    m_joints.push_back(joint);
    // -350, 350
    joint = Joint(-6.10865f, 6.10865f, modelFactoryPqp.createModel("meshes/KukaKR5/link4.stl"));
    m_joints.push_back(joint);
    // 50, 310
    joint = Joint(0.872665f, 5.41052f, modelFactoryPqp.createModel("meshes/KukaKR5/link5.stl"));
    m_joints.push_back(joint);
    // -360, 170
    joint = Joint(-util::twoPi(), 2.96706f, modelFactoryPqp.createModel("meshes/KukaKR5/link6.stl"));
    m_joints.push_back(joint);
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-10-22
*/
Vector6 KukaKR5::directKinematic(const Vector6 &angles)  const {
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
std::vector<Matrix4> KukaKR5::getJointTrafos(const Vector6 &angles)  const {
    Vector6 rads = util::degToRad<6>(angles);

    std::vector<Matrix4> trafos;
    for (size_t i = 0; i < 6; ++i) {
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]));
    }
    return trafos;
}

} /* namespace ippp */
