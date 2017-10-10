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

#ifndef UTILCOLLISION_HPP
#define UTILCOLLISION_HPP

#include <ippp/core/util/UtilGeo.hpp>
#include <ippp/environment/robot/SerialRobot.h>

namespace ippp {
namespace util {

static std::pair<std::vector<Matrix3>, std::vector<Vector3>> getTrafosFromRobot(const VectorX &vec,
                                                                                const std::shared_ptr<SerialRobot> &robot,
                                                                                Matrix3 &poseR, Vector3 &poseT) {
    std::vector<Matrix4> jointTrafos = robot->getJointTrafos(vec);
    Matrix4 pose = robot->getPoseMat();
	Matrix4 *As = new Matrix4[robot->getDim()];
    As[0] = pose * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i) {
        As[i] = As[i - 1] * jointTrafos[i];
    }
    util::decomposeT(pose, poseR, poseT);

    std::vector<Matrix3> rots(jointTrafos.size());
    std::vector<Vector3> trans(jointTrafos.size());
    for (int i = 0; i < jointTrafos.size(); ++i) {
        util::decomposeT(As[i], rots[i], trans[i]);
    }
	delete []As;
    return std::make_pair(rots, trans);
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILCOLLISION_HPP
