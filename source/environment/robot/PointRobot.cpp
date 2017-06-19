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

#include <ippp/environment/robot/PointRobot.h>

#include <ippp/core/util/Logging.h>

namespace ippp {

/*!
*  \brief      Constructor of the 2D PointRobot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
PointRobot::PointRobot(const std::pair<Vector2, Vector2> &boundary)
    : RobotBase("PointRobot", 2, RobotType::mobile, boundary, std::vector<DofType>({DofType::planarPos, DofType::planarPos})) {
}

} /* namespace ippp */
