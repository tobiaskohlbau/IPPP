//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <iostream>

#include <Eigen/Core>

#include <core/dataObj/Node.hpp>
#include <core/module/Identifier.h>
#include <robot/model/ModelContainer.h>
#include <robot/RobotBase.hpp>

namespace rmpl {

/*!
* \brief   Abstract class CollisionDetection, base class of all CollisionDetections
* \author  Sascha Kaden
* \date    2017-02-19
*/
template <unsigned int dim>
class CollisionDetection : public Identifier {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot);
    virtual bool controlVec(const Vector<dim> &vec) = 0;
    virtual bool controlTrajectory(std::vector<Vector<dim>> &vec) = 0;

  protected:
    std::shared_ptr<RobotBase<dim>> m_robot;
    Vector<dim> m_minBoundary, m_maxBoundary;

    std::shared_ptr<ModelContainer> m_workspace = nullptr;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  name
*  \param[in]  robot
*  \date       2017-02-19
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot) : Identifier(name) {
    m_robot = robot;
    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
