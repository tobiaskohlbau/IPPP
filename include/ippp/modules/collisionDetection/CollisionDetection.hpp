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

#include <ippp/modules/collisionDetection/CollisionRequest.h>
#include <ippp/modules/collisionDetection/CollisionResult.h>
#include <ippp/modules/validityChecker/ValidityChecker.hpp>
#include <ippp/statistic/Stats.h>
#include <ippp/statistic/StatsCollisionCollector.h>

namespace ippp {

/*!
* \brief   Abstract class CollisionDetection, base class of all CollisionDetections
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection : public ValidityChecker<dim> {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment,
                       const CollisionRequest &request = CollisionRequest());

    virtual bool check(const Vector<dim> &config, const CollisionRequest &request, CollisionResult &result) const = 0;

  protected:
    const CollisionRequest m_request; /*!< Default request for single collision tests (not trajectories) */
    std::shared_ptr<StatsCollisionCollector> m_collisionCollector = nullptr;

    using ValidityChecker<dim>::m_environment;
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
    : ValidityChecker<dim>(name, environment),
      m_request(request),
      m_collisionCollector(std::make_shared<StatsCollisionCollector>("CollisionCount")) {
    Stats::addCollector(m_collisionCollector);
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION_HPP */
