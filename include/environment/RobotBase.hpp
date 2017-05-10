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

#ifndef ROBOTBASE_HPP
#define ROBOTBASE_HPP

#include <string>
#include <vector>

#include <Eigen/Core>

#include <core/Identifier.h>
#include <core/types.h>
#include <core/utility/Logging.h>
#include <core/utility/Utility.h>
#include <environment/model/ModelContainer.h>

namespace ippp {

enum RobotType { serial, mobile };

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
template <unsigned int dim>
class RobotBase : public Identifier {
  public:
    virtual ~RobotBase();

  protected:
    RobotBase(const std::string &name, RobotType robotType, const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary);

  public:
    void setPose(const Vector6 &pose);
    Vector6 getPose();
    Matrix4 getPoseMat();

    void setBaseModel(const std::shared_ptr<ModelContainer> &baseModel);
    std::shared_ptr<ModelContainer> getBaseModel();

    void setWorkspace(const std::shared_ptr<ModelContainer> &model);
    std::shared_ptr<ModelContainer> getWorkspace();

    Vector<dim> getMinBoundary() const;
    Vector<dim> getMaxBoundary() const;
    unsigned int getDim() const;
    RobotType getRobotType() const;

  protected:
    const RobotType m_robotType;
    const Vector<dim> m_minBoundary;
    const Vector<dim> m_maxBoundary;

    Vector6 m_pose;
    Matrix4 m_poseMat;

    std::shared_ptr<ModelContainer> m_baseModel;
    std::shared_ptr<ModelContainer> m_workspaceModel;
};

/*!
*  \brief      Standard deconstructor of the RobotBase
*  \author     Sasch Kaden
*  \date       2016-12-23
*/
template <unsigned int dim>
RobotBase<dim>::~RobotBase() {
}

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
template <unsigned int dim>
RobotBase<dim>::RobotBase(const std::string &name, RobotType robotType, const Vector<dim> &minBoundary, const Vector<dim> &maxBoundary)
    : Identifier(name), m_robotType(robotType), m_minBoundary(minBoundary), m_maxBoundary(maxBoundary) {
    m_pose = util::Vecf(0, 0, 0, 0, 0, 0);
    m_poseMat = util::poseVecToMat(m_pose);
    m_baseModel = nullptr;
    m_workspaceModel = nullptr;
}

/*!
*  \brief      Set pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \date       2016-07-24
*/
template <unsigned int dim>
void RobotBase<dim>::setPose(const Vector6 &pose) {
    if (pose.rows() != 6) {
        Logging::error("Pose vector has wrong dimension, must have 6!", this);
        return;
    }

    m_pose = pose;
    m_poseMat = util::poseVecToMat(pose);
}

/*!
*  \brief      Get pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[out] pose Vec
*  \date       2016-07-24
*/
template <unsigned int dim>
Vector6 RobotBase<dim>::getPose() {
    return m_pose;
}

/*!
*  \brief      Get pose transformation matrix
*  \author     Sascha Kaden
*  \param[out] pose matrix
*  \date       2016-07-24
*/
template <unsigned int dim>
Matrix4 RobotBase<dim>::getPoseMat() {
    return m_poseMat;
}

/*!
*  \brief      Load cad models from passed vector of strings and save them intern
*  \author     Sascha Kaden
*  \param[in]  model
*  \date       2016-06-30
*/
template <unsigned int dim>
void RobotBase<dim>::setBaseModel(const std::shared_ptr<ModelContainer> &model) {
    if (model->empty()) {
        Logging::error("Empty base model", this);
        return;
    }
    m_baseModel = model;
}

/*!
*  \brief      Return base model
*  \author     Sascha Kaden
*  \param[out] base model
*  \date       2016-06-30
*/
template <unsigned int dim>
std::shared_ptr<ModelContainer> RobotBase<dim>::getBaseModel() {
    return m_baseModel;
}

/*!
*  \brief      Set workspace to robot
*  \author     Sascha Kaden
*  \param[in]  model
*  \date       2016-07-14
*/
template <unsigned int dim>
void RobotBase<dim>::setWorkspace(const std::shared_ptr<ModelContainer> &model) {
    if (model->empty()) {
        Logging::error("Empty workspace model", this);
        return;
    }
    m_workspaceModel = model;
}

/*!
*  \brief      Return workspace of robot
*  \author     Sascha Kaden
*  \param[out] pointer to workspace model
*  \date       2016-07-14
*/
template <unsigned int dim>
std::shared_ptr<ModelContainer> RobotBase<dim>::getWorkspace() {
    return m_workspaceModel;
}

/*!
*  \brief      Get minimum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] minimum Boudaries
*  \date       2016-07-15
*/
template <unsigned int dim>
Vector<dim> RobotBase<dim>::getMinBoundary() const {
    return m_minBoundary;
}

/*!
*  \brief      Get maximum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] maximum Boudaries
*  \date       2016-07-15
*/
template <unsigned int dim>
Vector<dim> RobotBase<dim>::getMaxBoundary() const {
    return m_maxBoundary;
}

/*!
*  \brief      Return dimension from the robot
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-06-30
*/
template <unsigned int dim>
unsigned int RobotBase<dim>::getDim() const {
    return dim;
}

/*!
*  \brief      Return the RobotType
*  \author     Sascha Kaden
*  \param[out] RobotType
*  \date       2016-08-25
*/
template <unsigned int dim>
RobotType RobotBase<dim>::getRobotType() const {
    return m_robotType;
}

} /* namespace ippp */

#endif /* ROBOTBASE_HPP */
