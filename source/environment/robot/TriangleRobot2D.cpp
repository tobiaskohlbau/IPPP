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

#include <environment/robot/TriangleRobot2D.h>

namespace ippp {

/*!
*  \brief      Standard constructor of the 2D TriangleRobot
*  \author     Sascha Kaden
*  \date       2016-11-15
*/
TriangleRobot2D::TriangleRobot2D(const std::shared_ptr<ModelContainer> &triangleModel,
                                 const std::pair<Vector3, Vector3> &boundary)
    : RobotBase("TriangleRobot2D", 3, RobotType::mobile, boundary,
                std::vector<DofType>({DofType::planarPos, DofType::planarPos, DofType::planarRot})) {
    setBaseModel(triangleModel);
}

} /* namespace ippp */
