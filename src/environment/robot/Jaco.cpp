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

#include <ippp/environment/robot/Jaco.h>

#include <ippp/core/util/Utility.h>

namespace ippp {

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
Jaco::Jaco() : SerialRobot("Jaco", 6, std::make_pair(util::Vecd(0, 42, 17, 0, 0, 0),
                                                     util::Vecd(360, 318, 343, 360, 360, 360)),
                           std::vector<DofType>({DofType::volumetricPos, DofType::volumetricPos, DofType::volumetricPos,
                                                 DofType::volumetricRot, DofType::volumetricRot, DofType::volumetricRot})) {
    m_alpha = util::Vecd(util::pi() / 2, util::pi(), util::pi() / 2, 0.95993f, 0.95993f, util::pi());
    m_a = util::Vecd(0, 410, 0, 0, 0, 0);
    m_d = util::Vecd(275.5f, 0, -9.8f, -249.18224f, -83.76448f, -210.58224f);

    m_pose = util::Vecd(0, 0, 0, 0, 0, 0);

    ModelFactoryPqp modelFactoryPqp;
    m_baseModel = modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_base.dae");

    m_joints.push_back(Joint(0, 360, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_1.dae")));
    m_joints.push_back(Joint(42, 318, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_2.dae")));
    m_joints.push_back(Joint(17, 343, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_3.dae")));
    m_joints.push_back(Joint(0, 360, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_4.dae")));
    m_joints.push_back(Joint(0, 360, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_5.dae")));
    m_joints.push_back(Joint(0, 360, modelFactoryPqp.createModel("meshes/Jaco/jaco2_link_5.dae")));
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-08-25
*/
Vector6 Jaco::directKinematic(const VectorX &angles)  const {
    std::vector<Matrix4> trafos = getJointTrafos(angles);

    return getTcpPosition(trafos);
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-07-14
*/
std::vector<Matrix4> Jaco::getJointTrafos(const VectorX &angles)  const {
    // transform form jaco physical angles to dh angles
    Vector6 dhAngles = convertRealToDH(angles);

    std::vector<Matrix4> trafos;
    // create transformation matrizes
    for (size_t i = 0; i < 6; ++i) {
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], dhAngles[i]));
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
Vector6 Jaco::convertRealToDH(const Vector6 &realAngles) const{
    Vector6 dhAngles(realAngles);
    dhAngles[0] = -realAngles[0];
    dhAngles[1] = realAngles[1] - util::halfPi();
    dhAngles[2] = realAngles[2] + util::halfPi();
    dhAngles[4] = realAngles[4] - util::pi();
    dhAngles[5] = realAngles[5] + util::pi();

    return dhAngles;
}

} /* namespace ippp */
