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

#ifndef COLLISIONRESULT_H
#define COLLISIONRESULT_H

#include <vector>

namespace ippp {

struct CollisionResult {
    bool collision = false;

    double penetrationDepth = std::numeric_limits<double>::max();
    double minDist = std::numeric_limits<double>::max();
    double minObstacleDist = std::numeric_limits<double>::max();
    double minRobotDist = std::numeric_limits<double>::max();
};

} /* namespace ippp */

#endif    // COLLISIONRESULT_H
