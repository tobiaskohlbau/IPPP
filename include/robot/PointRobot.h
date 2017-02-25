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

#ifndef POINTROBOT_H
#define POINTROBOT_H

#include <robot/RobotBase.hpp>
#include <robot/model/Model2D.h>

namespace rmpl {

/*!
* \brief   Class for the 2D point robot
* \author  Sascha Kaden
* \date    2016-06-30
*/
class PointRobot : public RobotBase<2> {
  public:
    PointRobot(const Vector2 &minBoundary, const Vector2 &maxBoundary);

  private:
};

} /* namespace rmpl */

#endif /* POINTROBOT_H */
