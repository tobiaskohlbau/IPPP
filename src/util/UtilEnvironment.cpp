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

#include <ippp/util/UtilEnvironment.hpp>

namespace ippp {
namespace util {

void saveMeshes(const Environment &env, const VectorX config, const std::string &prefix) {
    env.saveObstacleMeshes(prefix);
    auto robot = env.getRobot();
    robot->saveRobotMesh(config, prefix);
}

} /* namespace util */
} /* namespace ippp */


