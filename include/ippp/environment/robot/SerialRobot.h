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

#ifndef SERIALROBOT_HPP
#define SERIALROBOT_HPP

#include <ippp/environment/robot/Joint.h>
#include <ippp/environment/robot/RobotBase.h>

namespace ippp {

/*!
* \brief   Base class of all serial robots
* \author  Sascha Kaden
* \date    2016-08-25
*/
class SerialRobot : public RobotBase {
  public:
    SerialRobot(const std::string &name, const unsigned int dim, const std::pair<VectorX, VectorX> &boundary,
                const std::vector<DofType> &dofTypes);

    virtual Transform getTransformation(const VectorX &config) const;
    virtual Transform directKinematic(const VectorX &angles) const = 0;
    virtual std::vector<Transform> getJointTrafos(const VectorX &angles) const = 0;
    std::vector<Transform> getLinkTrafos(const VectorX &angles) const;
    Transform getTrafo(double alpha, double a, double d, double q) const;
    Transform getTcp(const std::vector<Transform> &trafos) const;

    void setBaseOffset(const Vector6 &baseOffset);
    void setBaseOffset(const Transform &baseOffset);
    Transform getBaseOffset() const;
    void setJoints(const std::vector<Joint> &joints);
    size_t getNbJoints() const;

    std::shared_ptr<ModelContainer> getModelFromJoint(const size_t jointIndex) const;
    std::vector<std::shared_ptr<ModelContainer>> getJointModels() const;

    void saveMeshConfig(const VectorX &angles);
    void saveMeshConfig(const std::vector<Transform> &As);

  protected:
    std::vector<Joint> m_joints;
    VectorX m_alpha;
    VectorX m_a;
    VectorX m_d;
    Transform m_baseOffset;
};

} /* namespace ippp */

#endif    // SERIALROBOT_HPP
