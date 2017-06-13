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

#include <environment/CadProcessing.h>
#include <environment/robot/Joint.h>
#include <environment/robot/RobotBase.h>

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

    virtual Vector6 directKinematic(const VectorX &angles) = 0;
    virtual std::vector<Matrix4> getJointTrafos(const VectorX &angles) = 0;
    Matrix4 getTrafo(double alpha, double a, double d, double q);
    Vector6 getTcpPosition(const std::vector<Matrix4> &trafos);

    void setJoints(std::vector<Joint> joints);
    unsigned int getNbJoints();

    std::shared_ptr<ModelContainer> getModelFromJoint(unsigned int jointIndex);
    std::vector<std::shared_ptr<ModelContainer>> getJointModels();

    void saveMeshConfig(VectorX angles);
    void saveMeshConfig(Matrix4 *As);

  protected:
    std::vector<Joint> m_joints;
    VectorX m_alpha;
    VectorX m_a;
    VectorX m_d;
};

} /* namespace ippp */

#endif    // SERIALROBOT_HPP
