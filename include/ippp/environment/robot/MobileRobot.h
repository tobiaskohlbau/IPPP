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

#ifndef MOBILEROBOT_H
#define MOBILEROBOT_H

#include <ippp/environment/robot/RobotBase.h>

namespace ippp {

/*!
* \brief   Base class of all mobile robots.
* \details In the standard case it is interpreted as six dimensional robot inside of 3D space.
* \author  Sascha Kaden
* \date    2017-11-10
*/
class MobileRobot : public RobotBase {
  public:
    MobileRobot(unsigned int dim, const std::pair<VectorX, VectorX> &boundary, const std::vector<DofType> &dofTypes,
                const std::string &name = "MobileRobot");
    Transform getTransformation(const VectorX &config) const override;
};

} /* namespace ippp */

#endif    // MOBILEROBOT_H
