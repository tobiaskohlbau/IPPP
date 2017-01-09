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

#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <PQP.h>

#include <core/module/ModuleBase.h>
#include <core/types.h>
#include <core/utility/Logging.h>
#include <core/utility/Utility.h>
#include <robot/MeshContainer.h>

namespace rmpl {

enum CollisionType { fcl, pqp, point2D, triangle2D };

enum RobotType { serial, mobile };

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
template <unsigned int dim>
class RobotBase : public ModuleBase {
  public:
    virtual ~RobotBase();

  protected:
    RobotBase(std::string name, CollisionType collisionType, RobotType robotType, Vector<dim> minBoundary,
              Vector<dim> maxBoundary);

  public:
    void setPose(const Vector6 &pose);
    Vector6 getPose();
    Matrix4 getPoseMat();

    void setBaseMesh(const std::shared_ptr<MeshContainer> &baseMesh);
    std::shared_ptr<MeshContainer> getBaseMesh();

    void setWorkspace(const std::shared_ptr<MeshContainer> &mesh);
    std::shared_ptr<MeshContainer> getWorkspace();
    void set2DWorkspace(const Eigen::MatrixXi &space);
    Eigen::MatrixXi &get2DWorkspace();

    Vector<dim> getMinBoundary() const;
    Vector<dim> getMaxBoundary() const;
    unsigned int getDim() const;
    RobotType getRobotType() const;
    CollisionType getCollisionType() const;

  protected:
    const CollisionType m_collisionType;
    const RobotType m_robotType;
    const Vector<dim> m_minBoundary;
    const Vector<dim> m_maxBoundary;

    Vector6 m_pose;
    Matrix4 m_poseMat;

    std::shared_ptr<MeshContainer> m_baseMesh;
    std::shared_ptr<MeshContainer> m_workspaceMesh;
    Eigen::MatrixXi m_2DWorkspace;
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
RobotBase<dim>::RobotBase(std::string name, CollisionType collisionType, RobotType robotType, Vector<dim> minBoundary,
                          Vector<dim> maxBoundary)
    : ModuleBase(name),
      m_collisionType(collisionType),
      m_robotType(robotType),
      m_minBoundary(minBoundary),
      m_maxBoundary(maxBoundary) {
    m_pose = utilVec::Vecf(0, 0, 0, 0, 0, 0);
    m_poseMat = utilGeo::poseVecToMat(m_pose);
    m_baseMesh = nullptr;
    m_workspaceMesh = nullptr;
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
    m_poseMat = utilGeo::poseVecToMat(pose);
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
*  \param[in]  vector of file strings
*  \param[out] true if loading was feasible
*  \date       2016-06-30
*/
template <unsigned int dim>
void RobotBase<dim>::setBaseMesh(const std::shared_ptr<MeshContainer> &mesh) {
    m_baseMesh = mesh;
}

/*!
*  \brief      Return PQP cad model from index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] PQP cad model
*  \date       2016-06-30
*/
template <unsigned int dim>
std::shared_ptr<MeshContainer> RobotBase<dim>::getBaseMesh() {
    return m_baseMesh;
}

/*!
*  \brief      Set workspace to robot
*  \author     Sascha Kaden
*  \param[in]  file of workspace cad
*  \date       2016-07-14
*/
template <unsigned int dim>
void RobotBase<dim>::setWorkspace(const std::shared_ptr<MeshContainer> &mesh) {
    m_workspaceMesh = mesh;
}

/*!
*  \brief      Return workspace of robot
*  \author     Sascha Kaden
*  \param[out] pointer to PQP_Model
*  \date       2016-07-14
*/
template <unsigned int dim>
std::shared_ptr<MeshContainer> RobotBase<dim>::getWorkspace() {
    return m_workspaceMesh;
}

/*!
*  \brief      Set 2D workspace to robot
*  \author     Sascha Kaden
*  \param[in]  2D workspace
*  \date       2016-07-14
*/
template <unsigned int dim>
void RobotBase<dim>::set2DWorkspace(const Eigen::MatrixXi &workspace) {
    m_2DWorkspace = workspace;
}

/*!
*  \brief      Return 2D workspace of robot
*  \author     Sascha Kaden
*  \param[out] 2D workspace
*  \date       2016-07-14
*/
template <unsigned int dim>
Eigen::MatrixXi &RobotBase<dim>::get2DWorkspace() {
    return m_2DWorkspace;
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

/*!
*  \brief      Return the collision type of the robot
*  \author     Sascha Kaden
*  \param[out] CollisionType
*  \date       2016-06-30
*/
template <unsigned int dim>
CollisionType RobotBase<dim>::getCollisionType() const {
    return m_collisionType;
}

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
