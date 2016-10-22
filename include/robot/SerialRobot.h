//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#ifndef SERIALROBOT_H_
#define SERIALROBOT_H_

#include <robot/Joint.h>
#include <robot/RobotBase.h>

namespace rmpl {

/*!
* \brief   Base class of all serial robots
* \author  Sascha Kaden
* \date    2016-08-25
*/
class SerialRobot : public RobotBase {
  public:
    SerialRobot(std::string name, CollisionType type, unsigned int dim);

    virtual Vec<float> directKinematic(const Vec<float> &angles) = 0;
    virtual std::vector<Eigen::Matrix4f> getJointTrafos(const Vec<float> &angles) = 0;
    Eigen::Matrix4f getTrafo(float alpha, float a, float d, float q);
    Vec<float> getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos);

    void setJoints(std::vector<Joint> joints);
    unsigned int getNbJoints();

    std::shared_ptr<MeshContainer> getMeshFromJoint(unsigned int jointIndex);
    std::vector<std::shared_ptr<MeshContainer>> getJointMeshs();
    std::vector<std::shared_ptr<PQP_Model>> getJointPqpModels();
    std::vector<std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>>> getJointFclModels();

    void saveMeshConfig(Vec<float> angles);
    void saveMeshConfig(Eigen::Matrix4f *As);

  protected:
    std::vector<Joint> m_joints;
    Vec<float> m_alpha;
    Vec<float> m_a;
    Vec<float> m_d;
};

} /* namespace rmpl */

#endif    // SERIALROBOT_H_
