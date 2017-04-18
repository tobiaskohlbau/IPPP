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

#ifndef UTILCOLLISION_HPP
#define UTILCOLLISION_HPP

#include <core/utility/UtilGeo.hpp>
#include <robot/SerialRobot.hpp>

namespace rmpl {
namespace util {

template <unsigned int dim>
void getTrafosFromRobot(const Vector<dim> &vec, const std::shared_ptr<SerialRobot<dim>> &robot, Matrix3 &poseR,
                               Vector3 &poseT, Matrix3 (&Rs)[dim], Vector3 (&ts)[dim]) {
    std::vector<Matrix4> jointTrafos = robot->getJointTrafos(vec);
    Matrix4 pose = robot->getPoseMat();
    Matrix4 As[dim];
    As[0] = pose * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i) {
        As[i] = As[i - 1] * jointTrafos[i];
    }
    util::decomposeT(pose, poseR, poseT);

    for (int i = 0; i < jointTrafos.size(); ++i) {
        util::decomposeT(As[i], Rs[i], ts[i]);
    }
}

} /* namespace util */
} /* namespace rmpl */

#endif    // UTILCOLLISION_HPP