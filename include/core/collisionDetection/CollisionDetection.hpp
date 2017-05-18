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

#include <core/dataObj/Node.hpp>
#include <environment/Environment.h>

namespace ippp {

/*!
* \brief   Abstract class CollisionDetection, base class of all CollisionDetections
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection : public Identifier {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment);
    virtual bool controlVec(const Vector<dim> &vec) = 0;
    virtual bool controlTrajectory(std::vector<Vector<dim>> &vec) = 0;

  protected:
    const std::shared_ptr<Environment> m_environment;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::string &name, const std::shared_ptr<Environment> &environment)
    : Identifier(name), m_environment(environment) {
}

} /* namespace ippp */

#endif /* COLLISIONDETECTION_HPP */
