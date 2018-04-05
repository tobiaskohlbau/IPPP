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

#ifndef ROBOTBASE_H
#define ROBOTBASE_H

#include <memory>
#include <string>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/environment/EnvObject.h>
#include <ippp/environment/model/ModelContainer.h>
#include <ippp/types.h>
#include <ippp/util/Logging.h>

namespace ippp {

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public EnvObject {
  public:
    virtual ~RobotBase();

  protected:
    RobotBase(const std::string &name, unsigned int dim, RobotCategory robotType, std::vector<DofType> dofTypes);

  public:
    bool isStatic() const;
    virtual Transform getTransformation(const VectorX &config) const = 0;

    void setBaseModel(const std::shared_ptr<ModelContainer> &model);
    std::shared_ptr<ModelContainer> getBaseModel() const;

    std::pair<VectorX, VectorX> getBoundary() const;
    unsigned int getDim() const;
    RobotCategory getRobotCategory() const;
    std::vector<DofType> getDofTypes() const;

  protected:
    const RobotCategory m_robotType;        /*!< category of the robot (serial or mobile) */
    std::pair<VectorX, VectorX> m_boundary; /*!< robot boundary (first min, second max) */

    const unsigned int m_dim;              /*!< dimension of the robot */
    const std::vector<DofType> m_dofTypes; /*!< list of the types of every dimension  */

    std::shared_ptr<ModelContainer>
        m_baseModel; /*!< Basis model of the robot. For the mobile robot it is the model of the complete robot. */
};

} /* namespace ippp */

#endif /* ROBOTBASE_H */
