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
#include <robot/RobotBase.hpp>

namespace rmpl {

/*!
* \brief   Base class of all serial robots
* \author  Sascha Kaden
* \date    2016-08-25
*/
template <unsigned int dim>
class SerialRobot : public RobotBase<dim> {
  public:
    SerialRobot(std::string name, CollisionType type, Vector<dim> minBoundary, Vector<dim> maxBoundary);

    virtual Vector<dim> directKinematic(const Vector<dim> &angles) = 0;
    virtual std::vector<Matrix4> getJointTrafos(const Vector<dim> &angles) = 0;
    Matrix4 getTrafo(float alpha, float a, float d, float q);
    Vector6 getTcpPosition(const std::vector<Matrix4> &trafos);

    void setJoints(std::vector<Joint> joints);
    unsigned int getNbJoints();

    std::shared_ptr<MeshContainer> getMeshFromJoint(unsigned int jointIndex);
    std::vector<std::shared_ptr<MeshContainer>> getJointMeshs();
    std::vector<std::shared_ptr<PQP_Model>> getJointPqpModels();
    std::vector<std::shared_ptr<FCLModel>> getJointFclModels();

    void saveMeshConfig(Vector<dim> angles);
    void saveMeshConfig(Matrix4 *As);

  protected:
    std::vector<Joint> m_joints;
    Vector<dim> m_alpha;
    Vector<dim> m_a;
    Vector<dim> m_d;
};

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
template <unsigned int dim>
SerialRobot<dim>::SerialRobot(std::string name, CollisionType type, Vector<dim> minBoundary, Vector<dim> maxBoundary)
    : RobotBase<dim>(name, type, RobotType::serial, minBoundary, maxBoundary) {
}

/*!
*  \brief      Create transformation matrix from the passed D-H parameter and the joint angle
*  \author     Sascha Kaden
*  \param[in]  D-H alpha parameter
*  \param[in]  D-H a parameter
*  \param[in]  D-H d parameter
*  \param[in]  joint angle
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
template <unsigned int dim>
Matrix4 SerialRobot<dim>::getTrafo(float alpha, float a, float d, float q) {
    float sinAlpha = sin(alpha);
    float cosAlpha = cos(alpha);
    float sinQ = sin(q);
    float cosQ = cos(q);

    Matrix4 T = Eigen::Matrix4f::Zero(4, 4);
    T(0, 0) = cosQ;
    T(0, 1) = -sinQ * cosAlpha;
    T(0, 2) = sinQ * sinAlpha;
    T(0, 3) = a * cosQ;
    T(1, 0) = sinQ;
    T(1, 1) = cosQ * cosAlpha;
    T(1, 2) = -cosQ * sinAlpha;
    T(1, 3) = a * sinQ;
    T(2, 1) = sinAlpha;
    T(2, 2) = cosAlpha;
    T(2, 3) = d;
    T(3, 3) = 1;

    return T;
    // secont method for transformation matrix
    //    T(0, 0) = cosQ;
    //    T(0, 1) = -sinQ;
    //    T(0, 2) = 0;
    //    T(0, 3) = a;
    //    T(1, 0) = sinQ * cosAlpha;
    //    T(1, 1) = cosQ * cosAlpha;
    //    T(1, 2) = -sinAlpha;
    //    T(1, 3) = -sinAlpha * d;
    //    T(2,0) = sinQ * sinAlpha;
    //    T(2, 1) = cosQ * sinAlpha;
    //    T(2, 2) = cosAlpha;
    //    T(2, 3) = cosAlpha * d;
    //    T(3, 3) = 1;
}

/*!
*  \brief      Compute TCP pose from transformation matrizes and the basis pose
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \param[in]  basis pose
*  \param[out] TCP pose
*  \date       2016-07-07
*/
template <unsigned int dim>
Vector6 SerialRobot<dim>::getTcpPosition(const std::vector<Matrix4> &trafos) {
    // multiply these matrizes together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Matrix4 robotToTcp = trafos[0];
    for (int i = 1; i < 6; ++i)
        robotToTcp *= trafos[i];

    Matrix4 basisToTcp = this->m_poseMat * robotToTcp;

    return utilGeo::poseMatToVec(basisToTcp);
}

/*!
*  \brief      Return MeshContainer from joint by index
*  \author     Sascha Kaden
*  \param[in]  joint index
*  \param[out] MeshContainer
*  \date       2016-08-25
*/
template <unsigned int dim>
std::shared_ptr<MeshContainer> SerialRobot<dim>::getMeshFromJoint(unsigned int jointIndex) {
    if (jointIndex < m_joints.size()) {
        return m_joints[jointIndex].getMesh();
    } else {
        Logging::error("Joint index larger than joint size", this);
        return nullptr;
    }
}

/*!
*  \brief      Return MeshContainer vector from joints
*  \author     Sascha Kaden
*  \param[out] vector of MeshContainer
*  \date       2016-08-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<MeshContainer>> SerialRobot<dim>::getJointMeshs() {
    std::vector<std::shared_ptr<MeshContainer>> models;
    for (auto joint : m_joints)
        models.push_back(joint.getMesh());
    return models;
}

/*!
*  \brief      Return pqp model vector from joints
*  \author     Sascha Kaden
*  \param[out] vector of pqp models
*  \date       2016-08-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<PQP_Model>> SerialRobot<dim>::getJointPqpModels() {
    std::vector<std::shared_ptr<PQP_Model>> models;
    for (auto joint : m_joints)
        models.push_back(joint.getMesh()->getPqp());
    return models;
}

/*!
*  \brief      Return fcl model vector from joints
*  \author     Sascha Kaden
*  \param[out] vector of fcl models
*  \date       2016-08-25
*/
template <unsigned int dim>
std::vector<std::shared_ptr<FCLModel>> SerialRobot<dim>::getJointFclModels() {
    std::vector<std::shared_ptr<FCLModel>> models;
    for (auto joint : m_joints)
        models.push_back(joint.getMesh()->getFcl());
    return models;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
template <unsigned int dim>
unsigned int SerialRobot<dim>::getNbJoints() {
    return m_joints.size();
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  joint angles
*  \date       2016-10-22
*/
template <unsigned int dim>
void SerialRobot<dim>::saveMeshConfig(Vector<dim> angles) {
    std::vector<Matrix4> jointTrafos = getJointTrafos(angles);
    Matrix4 As[jointTrafos.size()];
    As[0] = this->m_poseMat * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i)
        As[i] = As[i - 1] * jointTrafos[i];

    saveMeshConfig(As);
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \date       2016-10-22
*/
template <unsigned int dim>
void SerialRobot<dim>::saveMeshConfig(Matrix4 *As) {
    if (this->m_baseMesh != nullptr)
        this->m_baseMesh->saveObj("base.obj", this->m_poseMat);

    for (int i = 0; i < dim; ++i) {
        getMeshFromJoint(i)->saveObj("link" + std::to_string(i) + ".obj", As[i]);
        // std::cout<< As[i] << std::endl <<std::endl;
    }
}

} /* namespace rmpl */

#endif    // SERIALROBOT_H_
