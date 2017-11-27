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

#ifndef SERIALROBOT2D_H
#define SERIALROBOT2D_H

#include <ippp/environment/robot/SerialRobot.h>

namespace ippp {

/*!
* \brief   Class for the Kuka KR5 robot
* \author  Sascha Kaden
* \date    2016-10-22
*/
class SerialRobot2D : public SerialRobot {
  public:
    SerialRobot2D();
    Transform directKinematic(const VectorX &angles) const override;
    std::vector<Transform> getJointTrafos(const VectorX &angles) const override;
};

} /* namespace ippp */

#endif /* SERIALROBOT2D_H */
