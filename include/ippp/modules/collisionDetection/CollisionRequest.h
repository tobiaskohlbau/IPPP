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

#ifndef COLLISIONREQUEST_H
#define COLLISIONREQUEST_H

#include <vector>

namespace ippp {

/*!
* \brief   Request for the CollisionDetection class and single collision method requests.
* \author  Sascha Kaden
* \date    2017-11-10
*/
struct CollisionRequest {
    bool checkInterRobot = true;          /*!< check inner collision of the robot */
    bool checkObstacle = true;            /*!< check collision to the obstacles */
    bool completeCheck = false;
    bool computeDistance = false;         /*!< compute the distance to obstacles and inner robot collisions */
    bool computePenetrationDepth = false; /*!< compute depth of collisions */
};

} /* namespace ippp */

#endif    // COLLISIONREQUEST_H
