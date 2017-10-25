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

#include <ippp/environment/robot/RobotBase.h>

namespace ippp {

/*!
*  \brief      Standard deconstructor of the RobotBase
*  \author     Sasch Kaden
*  \date       2016-12-23
*/
RobotBase::~RobotBase() {
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
RobotBase::RobotBase(const std::string &name, const unsigned int dim, RobotCategory robotType, const std::pair<VectorX, VectorX> &boundary,
                     const std::vector<DofType> &dofTypes)
    : Identifier(name), m_dim(dim), m_robotType(robotType), m_minBoundary(boundary.first), m_maxBoundary(boundary.second), m_dofTypes(dofTypes) {
    if (dim != m_dofTypes.size())
        Logging::error("DoF Types have not the size of the robot dimension", this);
    assert(dim == m_dofTypes.size());
    m_pose = util::Vecd(0, 0, 0, 0, 0, 0);
    m_poseMat = util::poseVecToMat(m_pose);
    m_baseModel = nullptr;
}

/*!
*  \brief      Set pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \date       2016-07-24
*/
void RobotBase::setPose(const Vector6 &pose) {
    if (pose.rows() != 6) {
        Logging::error("Pose vector has wrong dimension, must have 6!", this);
        return;
    }
    m_baseModel->transformModel(pose);

    m_pose = pose;
    m_poseMat = util::poseVecToMat(pose);
}

/*!
*  \brief      Get pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[out] pose Vec
*  \date       2016-07-24
*/
Vector6 RobotBase::getPose() const {
    return m_pose;
}

/*!
*  \brief      Get pose transformation matrix
*  \author     Sascha Kaden
*  \param[out] pose matrix
*  \date       2016-07-24
*/
Matrix4 RobotBase::getPoseMat() const {
    return m_poseMat;
}

/*!
*  \brief      Load cad models from passed vector of strings and save them intern
*  \author     Sascha Kaden
*  \param[in]  model
*  \date       2016-06-30
*/
void RobotBase::setBaseModel(const std::shared_ptr<ModelContainer> &model) {
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
std::shared_ptr<ModelContainer> RobotBase::getBaseModel() const {
    return m_baseModel;
}

/*!
*  \brief      Get minimum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] minimum Boudaries
*  \date       2016-07-15
*/
VectorX RobotBase::getMinBoundary() const {
    return m_minBoundary;
}

/*!
*  \brief      Get maximum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] maximum Boudaries
*  \date       2016-07-15
*/
VectorX RobotBase::getMaxBoundary() const {
    return m_maxBoundary;
}

/*!
*  \brief      Return dimension from the robot
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-06-30
*/
unsigned int RobotBase::getDim() const {
    return m_dim;
}

/*!
*  \brief      Return the RobotType
*  \author     Sascha Kaden
*  \param[out] RobotType
*  \date       2016-08-25
*/
RobotCategory RobotBase::getRobotCategory() const {
    return m_robotType;
}

/*!
*  \brief      Return the dofs of the robot
*  \author     Sascha Kaden
*  \param[out] dofs
*  \date       2017-06-20
*/
std::vector<DofType> RobotBase::getDofTypes() const {
    return m_dofTypes;
}

} /* namespace ippp */