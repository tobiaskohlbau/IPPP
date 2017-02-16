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

#include <Eigen/Core>

#include <core/dataObj/Node.hpp>
#include <core/module/ModuleBase.h>
#include <robot/MeshContainer.h>
#include <robot/RobotBase.hpp>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return binary value
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class CollisionDetection : public ModuleBase {
  public:
    CollisionDetection(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot);
    virtual bool controlVec(const Vector<dim> &vec) = 0;
    virtual bool controlTrajectory(std::vector<Vector<dim>> &vec) = 0;

  protected:
    std::shared_ptr<RobotBase<dim>> m_robot;
    Vector<dim> m_minBoundary, m_maxBoundary;

    Eigen::MatrixXi m_2DWorkspace;
    std::shared_ptr<MeshContainer> m_workspace = nullptr;
};

/*!
*  \brief      Constructor of the class CollisionDetection
*  \author     Sascha Kaden
*  \param[in]  VREP Helper
*  \param[in]  RobotType
*  \date       2016-06-30
*/
template <unsigned int dim>
CollisionDetection<dim>::CollisionDetection(const std::string &name, const std::shared_ptr<RobotBase<dim>> &robot) : ModuleBase("collisionDetection") {
    m_robot = robot;
    m_minBoundary = robot->getMinBoundary();
    m_maxBoundary = robot->getMaxBoundary();

    m_2DWorkspace = m_robot->get2DWorkspace();
    m_workspace = m_robot->getWorkspace();
}

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
