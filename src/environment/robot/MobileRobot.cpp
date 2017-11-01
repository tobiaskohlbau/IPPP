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

#include <ippp/environment/robot/MobileRobot.h>

namespace ippp {

MobileRobot::MobileRobot(const unsigned int dim, const std::pair<VectorX, VectorX> &boundary,
                         const std::vector<DofType> &dofTypes)
    : RobotBase("MobileRobot", dim, RobotCategory::mobile, boundary, dofTypes) {
}

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] pair with rotation and translation
*  \date       2017-06-21
*/
Transform MobileRobot::getTransformation(const VectorX &config) const {
    if (m_dim != 6)
        return Transform();
    else
        return util::poseVecToTransform(config);
}

} /* namespace ippp */
