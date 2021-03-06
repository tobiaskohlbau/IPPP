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

#include <ippp/environment/robot/SerialRobot.h>

#include <ippp/environment/cad/CadImportExport.h>
#include <ippp/environment/cad/CadProcessing.h>

namespace ippp {

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \date       2016-07-19
*/
SerialRobot::SerialRobot(const std::string &name, const unsigned int dim, const std::pair<VectorX, VectorX> &boundary,
                         const std::vector<DofType> &dofTypes)
    : RobotBase(name, dim, RobotCategory::serial, boundary, dofTypes) {
    m_baseOffset = Matrix4::Identity(4, 4);
}

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] Transform
*  \date       2017-06-21
*/
Transform SerialRobot::getTransformation(const VectorX &config) const {
    return getTcp(this->getJointTrafos(config));
}

/*!
*  \brief      Create transformation matrix from the passed D-H parameter and the joint angle
*  \author     Sascha Kaden
*  \param[in]  D-H alpha parameter
*  \param[in]  D-H a parameter
*  \param[in]  D-H d parameter
*  \param[in]  joint angle
*  \param[out] Transform
*  \date       2016-07-07
*/
Transform SerialRobot::getTrafo(double alpha, double a, double /*d*/, double q) const {
    // double sinAlpha = sin(alpha);
    // double cosAlpha = cos(alpha);
    // double sinQ = sin(q);
    // double cosQ = cos(q);

    // Matrix4 T = Matrix4::Zero(4, 4);
    // T(0, 0) = cosQ;
    // T(0, 1) = -sinQ * cosAlpha;
    // T(0, 2) = sinQ * sinAlpha;
    // T(0, 3) = a * cosQ;
    // T(1, 0) = sinQ;
    // T(1, 1) = cosQ * cosAlpha;
    // T(1, 2) = -cosQ * sinAlpha;
    // T(1, 3) = a * sinQ;
    // T(2, 1) = sinAlpha;
    // T(2, 2) = cosAlpha;
    // T(2, 3) = d;
    // T(3, 3) = 1;

    Transform T;
    T = Translation(Vector3(0, 0, 3)) * Eigen::AngleAxisd(q, Eigen::Vector3d::UnitZ()) * Translation(Vector3(a, 0, 0)) *
        Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitX());

    return T;
}

/*!
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of Transforms
*  \date       2016-10-22
*/
std::vector<Transform> SerialRobot::getLinkTrafos(const VectorX &angles) const {
    std::vector<Transform> jointTrafos = getJointTrafos(angles);
    std::vector<Transform> AsLink(jointTrafos.size());
    std::vector<Transform> AsJoint(jointTrafos.size());

    AsJoint[0] = m_pose * m_baseOffset * jointTrafos[0];
    AsLink[0] = m_pose * m_baseOffset;// *jointTrafos[0].translation();
    for (size_t i = 1; i < jointTrafos.size(); ++i) {
        AsJoint[i] = AsJoint[i - 1] * jointTrafos[i];
        AsLink[i] = AsJoint[i - 1];// *jointTrafos[i].translation();
    }
    return AsLink;
}

/*!
*  \brief      Compute tool center pose from transformation matrizes and the basis pose
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \param[out] TCP pose as Transform
*  \date       2016-07-07
*/
Transform SerialRobot::getTcp(const std::vector<Transform> &trafos) const {
    // multiply these matrizes together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Transform robotToTcp = trafos[0];
    for (size_t i = 1; i < 6; ++i)
        robotToTcp = robotToTcp * trafos[i];

    return this->m_pose * robotToTcp;
}

/*!
*  \brief      Return MeshContainer from joint by index
*  \author     Sascha Kaden
*  \param[in]  joint index
*  \param[out] shared_ptr of the ModelContainer
*  \date       2016-08-25
*/
std::shared_ptr<ModelContainer> SerialRobot::getModelFromJoint(const size_t jointIndex) const {
    if (jointIndex < m_joints.size()) {
        return m_joints[jointIndex].getModel();
    }
    Logging::error("Joint index larger than joint size", this);
    return nullptr;
}

/*!
*  \brief      Return MeshContainer vector from joints
*  \author     Sascha Kaden
*  \param[out] vector of ModelContainer
*  \date       2016-08-25
*/
std::vector<std::shared_ptr<ModelContainer>> SerialRobot::getJointModels() const {
    std::vector<std::shared_ptr<ModelContainer>> models;
    for (const auto &joint : m_joints)
        models.push_back(joint.getModel());
    return models;
}

/*!
*  \brief      Set the base offset (rotation and translation) of the base model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  configuration of the base offset
*  \date       2017-11-17
*/
void SerialRobot::setBaseOffset(const Vector6 &baseOffset) {
    setBaseOffset(util::poseVecToTransform(baseOffset));
}

/*!
*  \brief      Set the base offset (Transform) of the base model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  Transform of the base offset
*  \date       2017-11-17
*/
void SerialRobot::setBaseOffset(const Transform &baseOffset) {
    m_baseOffset = baseOffset;
}

/*!
*  \brief      Return the base offset (Transform) of the base model of the serial robot.
*  \author     Sascha Kaden
*  \param[out] Transform of the base offset
*  \date       2017-11-17
*/
Transform SerialRobot::getBaseOffset() const {
    return m_baseOffset;
}

/*!
*  \brief      Sets the joints of the serial robot, deletes at this step all old joints.
*  \author     Sascha Kaden
*  \param[in]  list of joints
*  \date       2017-11-17
*/
void SerialRobot::setJoints(const std::vector<Joint> &joints) {
    if (joints.empty())
        return;

    m_joints = joints;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
size_t SerialRobot::getNbJoints() const {
    return m_joints.size();
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  joint angles
*  \date       2016-10-22
*/
void SerialRobot::saveMeshConfig(const VectorX &angles) {
    std::vector<Transform> jointTrafos = getJointTrafos(angles);
    std::vector<Transform> As(jointTrafos.size());

    As[0] = m_pose * jointTrafos[0];
    for (size_t i = 1; i < jointTrafos.size(); ++i)
        As[i] = As[i - 1] * jointTrafos[i];

    saveMeshConfig(As);
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \date       2016-10-22
*/
void SerialRobot::saveMeshConfig(const std::vector<Transform> & /*As*/) {
    if (this->m_baseModel != nullptr) {
        std::vector<Vector3> verts;
        for (const auto &vertice : this->m_baseModel->m_mesh.vertices) {
            Vector4 temp(util::append<3>(vertice, (double)1));
            temp = this->m_pose * temp;
            verts.emplace_back(temp(0), temp(1), temp(2));
        }
        cad::exportCad(cad::ExportFormat::OBJ, "base", verts, this->m_baseModel->m_mesh.faces);
    }
    // this->m_baseModel->saveObj("base.obj", this->m_pose);

    for (size_t i = 0; i < m_joints.size(); ++i) {
        std::vector<Vector3> verts;
        for (const auto &vertex : getModelFromJoint(i)->m_mesh.vertices) {
            Vector4 temp(util::append<3>(vertex, (double)1));
            temp = this->m_pose * temp;
            verts.emplace_back(temp(0), temp(1), temp(2));
        }
        cad::exportCad(cad::ExportFormat::OBJ, "link" + std::to_string(i), verts, getModelFromJoint(i)->m_mesh.faces);
        // getModelFromJoint(i)->saveObj("link" + std::to_string(i) + ".obj",
        // As[i]); std::cout<< As[i] << std::endl <<std::endl;
    }
}

} /* namespace ippp */
