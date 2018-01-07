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

#include <utility>

namespace ippp {

/*!
*  \brief      Standard destructor of the RobotBase
*  \author     Sascha Kaden
*  \date       2016-12-23
*/
RobotBase::~RobotBase() = default;

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
RobotBase::RobotBase(const std::string &name, unsigned int dim, RobotCategory robotType, std::vector<DofType> dofTypes)
    : EnvObject(name, EnvObjectType::Robot),
      m_dim(dim),
      m_robotType(robotType),
      m_dofTypes(std::move(dofTypes)),
      m_baseModel(nullptr),
      m_pose(Transform::Identity()) {
    if (dim != m_dofTypes.size())
        Logging::error("DoF Types have not the size of the robot dimension", this);
    assert(dim == m_dofTypes.size());
}

/*!
*  \brief      Set pose of robot
*  \author     Sascha Kaden
*  \param[in]  Transform pose
*  \date       2017-11-01
*/
void RobotBase::setPose(const Vector6 &pose) {
    if (util::empty<6>(pose))
        Logging::warning("Set empty pose", this);

    m_pose = util::poseVecToTransform(pose);
}

/*!
*  \brief      Set pose of robot
*  \author     Sascha Kaden
*  \param[in]  Transform pose
*  \date       2017-11-01
*/
void RobotBase::setPose(const Transform &pose) {
    m_pose = pose;
}

/*!
*  \brief      Get pose of robot
*  \author     Sascha Kaden
*  \param[out] Transform pose
*  \date       2017-11-01
*/
Transform RobotBase::getPose() const {
    return m_pose;
}

/*!
*  \brief      Load cad models from passed vector of strings and save them intern
*  \author     Sascha Kaden
*  \param[in]  model
*  \date       2016-06-30
*/
void RobotBase::setBaseModel(const std::shared_ptr<ModelContainer> &model) {
    if (!model || model->empty()) {
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
*  \param[out] minimum Boundaries
*  \date       2016-07-15
*/
VectorX RobotBase::getMinBoundary() const {
    return m_minBoundary;
}

/*!
*  \brief      Get maximum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] maximum Boundaries
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
