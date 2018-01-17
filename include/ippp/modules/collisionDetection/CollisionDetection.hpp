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

#ifndef COLLISIONDETECTION_HPP
#define COLLISIONDETECTION_HPP

#include <Eigen/Core>

#include <ippp/dataObj/Node.hpp>
#include <ippp/environment/Environment.h>
#include <ippp/modules/collisionDetection/CollisionRequest.h>
#include <ippp/modules/collisionDetection/CollisionResult.h>

namespace ippp {

/*!
* \brief   Abstract class CollisionDetection, base class of all CollisionDetections
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection : public Identifier {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment,
                       const CollisionRequest &request = CollisionRequest());
    virtual bool checkConfig(const Vector<dim> &config, CollisionRequest *request = nullptr,
                             CollisionResult *result = nullptr) = 0;
    virtual bool checkTrajectory(const std::vector<Vector<dim>> &config) = 0;
    
  protected:
    const std::shared_ptr<Environment> m_environment;    /*!< Pointer to the Environment */
    const CollisionRequest m_request;                    /*!< Default request for single collision tests (not trajectories) */
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment,
                                            const CollisionRequest &request)
    : Identifier(name), m_environment(environment), m_request(request) {
    Logging::debug("Initialize", this);
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION_HPP */
