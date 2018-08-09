//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/environment/robot/MobileRobot.h>

#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  dimension
*  \param[in]  robot boundary
*  \param[out] dimension of freedom types of the robot
*  \date       2017-06-21
*/
MobileRobot::MobileRobot(unsigned int dim, const std::pair<VectorX, VectorX> &boundary, const std::vector<DofType> &dofTypes,
                         const std::string &name)
    : RobotBase(name, dim, RobotCategory::mobile, dofTypes) {
    m_boundary = boundary;
}

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] Transform
*  \date       2017-06-21
*/
Transform MobileRobot::getTransformation(const VectorX &config) const {
    if (m_dim != 6)
        return Transform::Identity();

    return util::toTransform(config);
}

} /* namespace ippp */
