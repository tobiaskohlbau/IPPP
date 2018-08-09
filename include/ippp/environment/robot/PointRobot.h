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

#ifndef POINTROBOT_H
#define POINTROBOT_H

#include <ippp/environment/robot/MobileRobot.h>

namespace ippp {

/*!
* \brief   Class of the planar point robot.
* \details This robot owns no body model. It has only two dimensions, moving in x or y direction.
* \author  Sascha Kaden
* \date    2016-06-30
*/
class PointRobot : public MobileRobot {
  public:
    PointRobot(const std::pair<Vector2, Vector2> &boundary);
    Transform getTransformation(const VectorX &config) const override;
};

} /* namespace ippp */

#endif /* POINTROBOT_H */
