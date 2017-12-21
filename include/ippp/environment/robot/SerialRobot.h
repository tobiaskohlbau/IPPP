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

#include <ippp/dataObj/DhParameter.h>
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
    SerialRobot(const unsigned int dim, const std::vector<Joint> &joints, const std::vector<DhParameter> &dhParameters,
                const std::vector<DofType> &dofTypes, const std::string &name = "SerialRobot");

    virtual Transform getTransformation(const VectorX &config) const;
    virtual std::vector<Transform> getJointTrafos(const VectorX &angles) const;
    std::vector<Transform> getLinkTrafos(const VectorX &angles) const;
    Transform getTrafo(const DhParameter &dhParam, double q) const;
    Transform getTcp(const std::vector<Transform> &trafos) const;

    void setBaseOffset(const Vector6 &baseOffset);
    void setBaseOffset(const Transform &baseOffset);
    void setLinkOffsets(const std::vector<Vector6> &offsets);
    void setLinkOffsets(const std::vector<Transform> &offsets);
    std::vector<Transform> getLinkOffsets() const;
    Transform getBaseOffset() const;
    size_t getNbJoints() const;

    std::shared_ptr<ModelContainer> getModelFromJoint(const size_t jointIndex) const;
    std::vector<std::shared_ptr<ModelContainer>> getJointModels() const;

    void saveMeshConfig(const VectorX &angles);

  protected:
    std::vector<Joint> m_joints;             /*!< joints of the serial robot */
    std::vector<DhParameter> m_dhParameters; /*!< dh parameter to the joints */
    Transform m_baseOffset;                  /*!< transformation offset of the base model, if used. */
    std::vector<Transform> m_linkOffsets;    /*!< extra offset transforms of the links */
};

} /* namespace ippp */

#endif    // SERIALROBOT_HPP
