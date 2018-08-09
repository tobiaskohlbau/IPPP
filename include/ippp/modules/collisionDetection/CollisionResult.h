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

#ifndef COLLISIONRESULT_H
#define COLLISIONRESULT_H

#include <vector>

namespace ippp {

/*!
* \brief   Result of all single collision requests.
* \details The result will never be filled completely, it varies with the used CollisionDetection.
* \author  Sascha Kaden
* \date    2017-11-10
*/
struct CollisionResult {
    bool collision = false;

    std::vector<std::pair<size_t, size_t>> interRobotCollisions;
    double penetrationDepth = std::numeric_limits<double>::max(); /*!< collision depth, valid if in collision */
    double minDist = std::numeric_limits<double>::max();          /*!< minimum distance to obstacles and robot bodies */
    double minObstacleDist = std::numeric_limits<double>::max();  /*!< minimum distance to obstacles */
    double minRobotDist = std::numeric_limits<double>::max();     /*!< minimum distance between robot bodies */
};

} /* namespace ippp */

#endif    // COLLISIONRESULT_H
