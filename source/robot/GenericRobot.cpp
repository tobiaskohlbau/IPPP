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

#include <robot/GenericRobot.h>

#include <include/core/utility/Logging.h>
#include <include/core/utility/Utility.h>

namespace rmpl {

/*!
*  \brief      Constructor of the GenericRobot robot
*  \author     Sascha Kaden
*  \date       2016-07-24
*/
GenericRobot::GenericRobot(std::string name, unsigned int dimension, const Eigen::VectorXf &alphaParams,
                           const Eigen::VectorXf &aParams, const Eigen::VectorXf dParams)
    : SerialRobot(name, CollisionType::pqp, dimension) {
    // check consistency of parameters
    if (alphaParams.rows() != dimension || aParams.rows() != dimension || dParams.rows() != dimension) {
        Logging::error("DH parameter have wrong dimensions, unequal to joint count!", this);
    }
}

Eigen::Matrix<float, 6, 1> GenericRobot::directKinematic(const Eigen::VectorXf &angles) {
    std::vector<Eigen::Matrix4f> trafos = getJointTrafos(angles);

    return getTcpPosition(trafos);
}

/*!
*  \brief      Get vector of transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-07-24
*/
std::vector<Eigen::Matrix4f> GenericRobot::getJointTrafos(const Eigen::VectorXf &angles) {
    Eigen::VectorXf rads = utility::degToRad(angles);

    std::vector<Eigen::Matrix4f> trafos;
    Eigen::Matrix4f A = Eigen::Matrix4f::Zero(4, 4);
    for (int i = 0; i < 4; ++i)
        A(i, i) = 1;
    trafos.push_back(A);

    // create transformation matrizes
    for (int i = 0; i < m_joints.size(); ++i) {
        A = getTrafo(m_alpha[i], m_a[i], m_d[i], rads[i]);
        trafos.push_back(A);
    }
    return trafos;
}

} /* namespace rmpl */