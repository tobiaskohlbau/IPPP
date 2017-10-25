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

#include <ippp/environment/robot/SerialRobot2D.h>

#include <ippp/core/util/Utility.h>
#include <ippp/environment/model/ModelFactoryTriangle2D.h>
#include <ippp/environment/model/ModelFactoryPQP.h>

namespace ippp {

/*!
*  \brief      Constructor of the Jaco robot
*  \details    Sets all parameter from the Jaco serial robot and loads the mesh models.
*  \author     Sascha Kaden
*  \date        2016-10-22
*/
SerialRobot2D::SerialRobot2D()
    : SerialRobot("SerialRobot2D", 5, std::make_pair(util::Vecd(0, 0, 0, 0, 0), util::Vecd(util::twoPi(), util::twoPi(), util::twoPi(),
                                                                                  util::twoPi(), util::twoPi())),
                  std::vector<DofType>(
                      {DofType::planarRot, DofType::planarRot, DofType::planarRot, DofType::planarRot, DofType::planarRot})) {
    m_alpha = util::Vecd(0, 0, 0, 0, 0);
    m_alpha = util::degToRad<5>(m_alpha);
    m_a = util::Vecd(100, 100, 100, 100, 100);
    m_d = util::Vecd(0, 0, 0, 0, 0);
    setPose(util::Vecd(0, 0, 0, 0, 0, 0));
    setBaseOffset(util::Vecd(100, 0, 0, 0, 0, 0));

    ModelFactoryPqp modelFactory;
    m_baseModel = modelFactory.createModel("assets/robotModels/2Dline.obj");
    Joint joint;
    joint = Joint(0, util::twoPi(), modelFactory.createModel("assets/robotModels/2Dline.obj"));
    m_joints.push_back(joint);
    joint = Joint(0, util::twoPi(), modelFactory.createModel("assets/robotModels/2Dline.obj"));
    m_joints.push_back(joint);
    joint = Joint(0, util::twoPi(), modelFactory.createModel("assets/robotModels/2Dline.obj"));
    m_joints.push_back(joint);
    joint = Joint(0, util::twoPi(), modelFactory.createModel("assets/robotModels/2Dline.obj"));
    m_joints.push_back(joint);
    joint = Joint(0, util::twoPi(), modelFactory.createModel("assets/robotModels/2Dline.obj"));
    m_joints.push_back(joint);
}

/*!
*  \brief      Computes the euclidean tcp position from the passed robot angles.
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] euclidean position Vec
*  \date       2016-10-22
*/
Vector6 SerialRobot2D::directKinematic(const VectorX &angles) const {
    return getTcpPosition(getJointTrafos(angles));
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-10-22
*/
std::vector<Matrix4> SerialRobot2D::getJointTrafos(const VectorX &angles) const {
    std::vector<Matrix4> trafos;
    for (size_t i = 0; i < getDim(); ++i)
        trafos.push_back(getTrafo(m_alpha[i], m_a[i], m_d[i], angles[i]));

    return trafos;
}

} /* namespace ippp */
