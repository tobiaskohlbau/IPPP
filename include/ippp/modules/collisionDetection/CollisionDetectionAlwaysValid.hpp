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

#ifndef COLLISIONDETECTIONALAWAYSVALID_HPP
#define COLLISIONDETECTIONALAWAYSVALID_HPP

#include <ippp/modules/collisionDetection/CollisionDetection.hpp>

namespace ippp {

/*!
* \brief   CollisionDetection dummy return always false, that means everything is free
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetectionAlwaysValid : public CollisionDetection<dim> {
  public:
    CollisionDetectionAlwaysValid(const std::shared_ptr<Environment> &environmen, const CollisionRequest &request = CollisionRequest());
    bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr, CollisionResult *result = nullptr);
    bool checkTrajectory(const std::vector<Vector<dim>> &config);
};

/*!
*  \brief      Standard constructor of the class CollisionDetectionDummy
*  \author     Sascha Kaden
*  \param[in]  Environment
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetectionAlwaysValid<dim>::CollisionDetectionAlwaysValid(const std::shared_ptr<Environment> &environment, const CollisionRequest &request)
        : CollisionDetection<dim>("CollisionDetectionAlwaysValid", environment, request) {

}

/*!
*  \brief      Check for collision
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] false
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionAlwaysValid<dim>::checkConfig(const Vector<dim> &config, CollisionRequest *reuest, CollisionResult *result) {
    return false;
}

/*!
*  \brief      Check collision of a trajectory of points
*  \author     Sascha Kaden
*  \param[in]  vector of configurations
*  \param[out] false
*  \date       2017-02-19
*/
template <unsigned int dim>
bool CollisionDetectionAlwaysValid<dim>::checkTrajectory(const std::vector<Vector<dim>> &configs) {
    return false;
}

} /* namespace ippp */

#endif /* COLLISIONDETECTIONALAWAYSVALID_HPP */
