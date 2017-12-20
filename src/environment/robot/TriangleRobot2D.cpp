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

#include <ippp/environment/robot/TriangleRobot2D.h>

#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
*  \brief      Standard constructor of the 2D TriangleRobot
*  \author     Sascha Kaden
*  \param[in]  ModelContainer with triangle model
*  \param[in]  robot boundary (min, max)
*  \date       2017-06-21
*/
TriangleRobot2D::TriangleRobot2D(const std::shared_ptr<ModelContainer> &triangleModel,
                                 const std::pair<Vector3, Vector3> &boundary)
    : MobileRobot(3, boundary, std::vector<DofType>({DofType::planarPos, DofType::planarPos, DofType::planarRot}),
                  "TriangleRobot2D") {
    setBaseModel(triangleModel);
}

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] Transform
*  \date       2017-06-21
*/
Transform TriangleRobot2D::getTransformation(const VectorX &config) const {
    Transform T;
    T = Translation(Vector3(config[0], config[1], 0)) * Eigen::AngleAxisd(config[2], Eigen::Vector3d::UnitZ());
    return T;
}

} /* namespace ippp */
