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

#include <robot/RobotBase.h>

#include <core/Logging.h>
#include <core/Utilities.h>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
RobotBase::RobotBase(std::string name, CollisionType collisionType, RobotType robotType, unsigned int dim) : ModuleBase(name) {
    m_collisionType = collisionType;
    m_robotType = robotType;
    m_dim = dim;

    m_pose = Vec<float>(0, 0, 0, 0, 0, 0);
    m_poseMat = Utilities::poseVecToMat(m_pose);
    m_baseMesh = nullptr;
    m_workspaceMesh = nullptr;
}

/*!
*  \brief      Get minimum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] minimum Boudaries
*  \date       2016-07-15
*/
Vec<float> RobotBase::getMinBoundary() {
    return m_minBoundary;
}

/*!
*  \brief      Get maximum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] maximum Boudaries
*  \date       2016-07-15
*/
Vec<float> RobotBase::getMaxBoundary() {
    return m_maxBoundary;
}

/*!
*  \brief      Set pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \date       2016-07-24
*/
void RobotBase::setPose(const Vec<float> &pose) {
    if (pose.getDim() != 6) {
        Logging::warning("Pose vector has wrong dimension, must have 6!", this);
        return;
    } else if (pose.empty()) {
        Logging::warning("Empty pose vector!", this);
        return;
    }

    m_pose = pose;
    m_poseMat = Utilities::poseVecToMat(pose);
}

/*!
*  \brief      Get pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[out] pose Vec
*  \date       2016-07-24
*/
Vec<float> RobotBase::getPose() {
    return m_pose;
}

/*!
*  \brief      Get pose transformation matrix
*  \author     Sascha Kaden
*  \param[out] pose matrix
*  \date       2016-07-24
*/
Eigen::Matrix4f RobotBase::getPoseMat() {
    return m_poseMat;
}

/*!
*  \brief      Load cad models from passed vector of strings and save them intern
*  \author     Sascha Kaden
*  \param[in]  vector of file strings
*  \param[out] true if loading was feasible
*  \date       2016-06-30
*/
void RobotBase::setBaseMesh(const std::shared_ptr<MeshContainer> &mesh) {
    m_baseMesh = mesh;
}

/*!
*  \brief      Return PQP cad model from index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] PQP cad model
*  \date       2016-06-30
*/
std::shared_ptr<MeshContainer> RobotBase::getBaseMesh() {
    return m_baseMesh;
}

/*!
*  \brief      Set workspace to robot
*  \author     Sascha Kaden
*  \param[in]  file of workspace cad
*  \date       2016-07-14
*/
void RobotBase::setWorkspace(const std::shared_ptr<MeshContainer> &mesh) {
    m_workspaceMesh = mesh;
}

/*!
*  \brief      Return workspace of robot
*  \author     Sascha Kaden
*  \param[out] pointer to PQP_Model
*  \date       2016-07-14
*/
shared_ptr<MeshContainer> RobotBase::getWorkspace() {
    return m_workspaceMesh;
}

/*!
*  \brief      Set 2D workspace to robot
*  \author     Sascha Kaden
*  \param[in]  2D workspace
*  \date       2016-07-14
*/
void RobotBase::set2DWorkspace(const Eigen::MatrixXi &workspace) {
    m_2DWorkspace = workspace;
}

/*!
*  \brief      Return 2D workspace of robot
*  \author     Sascha Kaden
*  \param[out] 2D workspace
*  \date       2016-07-14
*/
Eigen::MatrixXi &RobotBase::get2DWorkspace() {
    return m_2DWorkspace;
}

/*!
*  \brief      Return dimension from the robot
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-06-30
*/
unsigned int RobotBase::getDim() {
    return m_dim;
}

/*!
*  \brief      Return the RobotType
*  \author     Sascha Kaden
*  \param[out] RobotType
*  \date       2016-08-25
*/
RobotType RobotBase::getRobotType() {
    return m_robotType;
}

/*!
*  \brief      Set the collision type of the robot
*  \author     Sascha Kaden
*  \param[in]  CollisionType
*  \date       2016-07-24
*/
void RobotBase::setCollisionType(CollisionType type) {
    if (type == CollisionType::twoD && m_dim != 2) {
        Logging::warning("CollisionType twoD unequal to dimension", this);
    } else {
        m_collisionType = type;
    }
}

/*!
*  \brief      Return the collision type of the robot
*  \author     Sascha Kaden
*  \param[out] CollisionType
*  \date       2016-06-30
*/
CollisionType RobotBase::getCollisionType() {
    return m_collisionType;
}