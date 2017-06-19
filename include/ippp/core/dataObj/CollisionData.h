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

#ifndef COLLISIONDATA_H
#define COLLISIONDATA_H

#include <vector>

namespace ippp {

class CollisionData {
  public:
    bool collision = false;

    double penetrationDepth = 0;
    double minDist = 0;

    unsigned int numCollisions = 0;
    unsigned int triangleTests = 0;
    unsigned int boundingTests = 0;

    std::vector<unsigned int> robotsToCheck;
};

} /* namespace ippp */

#endif // COLLISIONDATA_H
