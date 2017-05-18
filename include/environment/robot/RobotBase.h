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
class RobotBase : public Identifier {
  public:
    virtual ~RobotBase();

  protected:
    RobotBase(const std::string &name, const unsigned int dim, RobotType robotType, const VectorX &minBoundary,
              const VectorX &maxBoundary);

  public:
    void setPose(const Vector6 &pose);
    Vector6 getPose();
    Matrix4 getPoseMat();

    void setBaseModel(const std::shared_ptr<ModelContainer> &baseModel);
    std::shared_ptr<ModelContainer> getBaseModel();

    VectorX getMinBoundary() const;
    VectorX getMaxBoundary() const;
    unsigned int getDim() const;
    RobotType getRobotType() const;

  protected:
    const RobotType m_robotType;
    const VectorX m_minBoundary;
    const VectorX m_maxBoundary;

    const unsigned int m_dim;
    Vector6 m_pose;
    Matrix4 m_poseMat;

    std::shared_ptr<ModelContainer> m_baseModel;
};

} /* namespace ippp */

#endif /* ROBOTBASE_HPP */
