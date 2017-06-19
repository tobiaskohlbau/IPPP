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

#include <ippp/core/Identifier.h>
#include <ippp/core/types.h>
#include <ippp/core/util/Logging.h>
#include <ippp/core/util/Utility.h>
#include <ippp/environment/model/ModelContainer.h>

namespace ippp {

enum RobotType { serial, mobile };
enum DofType { planarPos, planarRot, volumetricPos, volumetricRot, joint};

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Identifier {
  public:
    virtual ~RobotBase();

  protected:
    RobotBase(const std::string &name, const unsigned int dim, const RobotType robotType, const std::pair<VectorX, VectorX> &boundary,
              const std::vector<DofType> &dofTypes);

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
    const std::vector<DofType> m_dofTypes;
    Vector6 m_pose;
    Matrix4 m_poseMat;

    std::shared_ptr<ModelContainer> m_baseModel;
};

} /* namespace ippp */

#endif /* ROBOTBASE_HPP */
