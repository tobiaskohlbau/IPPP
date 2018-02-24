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

#include <ippp/util/UtilGeo.hpp>

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
      m_baseModel(nullptr) {
    if (dim != m_dofTypes.size())
        Logging::error("DoF Types have not the size of the robot dimension", this);
    assert(dim == m_dofTypes.size());
}

/*!
*  \brief      Return the static flag of the EnvObject
*  \author     Sascha Kaden
*  \param[out] false
*  \date       2018-01-10
*/
bool RobotBase::isStatic() const {
    return false;
}

/*!
*  \brief      Set the base model of the robot.
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
*  \brief      Get boundary of the robot (pair with min and max boundary).
*  \author     Sascha Kaden
*  \param[out] boundaries
*  \date       2018-02-15
*/
std::pair<VectorX, VectorX> RobotBase::getBoundary() const {
    return m_boundary;
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
