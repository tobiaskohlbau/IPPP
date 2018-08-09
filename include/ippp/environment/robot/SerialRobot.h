//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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
* \brief   Base class of all serial robots, it contains the direct kinematic with the DH parameter.
* \details The serial robot consists basically from a list of joints, furthermore a tool from the robot. The tool is split in two
* things, as first the model with the optional transformation offset and secondly the transformation from the last joint to the
* tool center point.
* \author  Sascha Kaden
* \date    2016-08-25
*/
class SerialRobot : public RobotBase {
  public:
    SerialRobot(unsigned int dim, const std::vector<Joint> &joints, const std::vector<DofType> &dofTypes,
                const std::string &name = "SerialRobot");

    virtual Transform getTransformation(const VectorX &config) const;
    virtual std::vector<Transform> getJointTrafos(const VectorX &angles) const;
    virtual std::vector<Transform> getLinkTrafos(const VectorX &angles) const;
    virtual std::pair<std::vector<Transform>, Transform> getLinkAndToolTrafos(const VectorX &angles) const;
    Transform getTrafo(const DhParameter &dhParam, double q) const;
    Transform getTcp(const std::vector<Transform> &trafos) const;
    MatrixX calcJacobian(const VectorX &config) const;

    void setBaseOffset(const Vector6 &baseOffset);
    void setBaseOffset(const Transform &baseOffset);
    Transform getBaseOffset() const;

    void setTcpOffset(const Vector6 &tcpOffset);
    void setTcpOffset(const Transform &tcpOffset);
    Transform getTcpOffset() const;

    void setToolModelOffset(const Vector6 &toolOffset);
    void setToolModelOffset(const Transform &toolOffset);
    Transform getToolModelOffset() const;
    void setToolModel(const std::shared_ptr<ModelContainer> &tool);
    std::shared_ptr<ModelContainer> getToolModel() const;

    size_t numJoints() const;
    std::shared_ptr<ModelContainer> getLinkModel(size_t index) const;
    std::vector<std::shared_ptr<ModelContainer>> getLinkModels() const;
    std::vector<Transform> getLinkOffsets() const;

    void saveRobotMesh(const VectorX &configuration, const std::string &prefix = "") const override;

  protected:
    void updateJointParams();

    std::vector<Joint> m_joints;             /*!< joints of the serial robot */
    std::vector<DhParameter> m_dhParameters; /*!< dh parameter to the joints */
    std::vector<Transform> m_linkOffsets;    /*!< extra offset transforms of the links */
    std::vector<std::shared_ptr<ModelContainer>> m_linkModels;
    std::vector<Vector3> m_zUnitVectors;

    Transform m_baseOffset = Transform::Identity(); /*!< transformation offset of the base model, if used. */
    Transform m_tcpOffset = Transform::Identity();
    Transform m_toolModelOffset = Transform::Identity();   /*!< transformation offset of the tool model, if used. */
    std::shared_ptr<ModelContainer> m_toolModel = nullptr; /*!< tool model at the last link */
};

} /* namespace ippp */

#endif    // SERIALROBOT_HPP
