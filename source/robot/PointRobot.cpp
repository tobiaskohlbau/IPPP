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

#include <robot/PointRobot.h>

using namespace rmpl;

/*!
*  \brief      Constructor of the 2D PointRobot
*  \author     Sascha Kaden
*  \date       2016-06-30
*/
PointRobot::PointRobot()
    : RobotBase("PointRobot", CollisionType::twoD, 2, 0) {

}

Vec<REAL> PointRobot::directKinematic(const Vec<REAL> &angles) {
    return angles;
}

std::vector<Eigen::Matrix4f> PointRobot::getTransformations(const Vec<REAL> &angles) {
    return std::vector<Eigen::Matrix4f>();
}
