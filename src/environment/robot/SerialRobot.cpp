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
#include <ippp/util/Logging.h>

namespace ippp {

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  minimum boundary
*  \param[in]  maximum boundary
*  \date       2016-07-19
*/
SerialRobot::SerialRobot(const unsigned int dim, const std::vector<Joint> &joints, const std::vector<DhParameter> &dhParameters,
                         const std::vector<DofType> &dofTypes, const std::string &name)
    : RobotBase(name, dim, RobotCategory::serial, dofTypes),
      m_baseOffset(Transform::Identity()),
      m_joints(joints),
      m_dhParameters(dhParameters),
      m_linkOffsets(dim, Transform::Identity()) {
    if (joints.size() != dim || dhParameters.size() != dim || dofTypes.size() != dim) {
        Logging::error("Dimension to parameter sizes is unequal", this);
        return;
    }

    m_minBoundary = VectorX::Zero(dim);
    m_maxBoundary = VectorX::Zero(dim);
    for (size_t i = 0; i < dim; ++i) {
        auto boundaries = joints[i].getBoundaries();
        m_minBoundary[i] = boundaries.first;
        m_maxBoundary[i] = boundaries.second;
    }
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
*  \brief      Get vector of Jaco transformation matrizes
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrizes
*  \date       2016-10-22
*/
std::vector<Transform> SerialRobot::getJointTrafos(const VectorX &angles) const {
    std::vector<Transform> trafos;
    for (size_t i = 0; i < m_dim; ++i)
        trafos.push_back(getTrafo(m_dhParameters[i], angles[i]));
    return trafos;
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
Transform SerialRobot::getTrafo(const DhParameter &dhParams, double q) const {
    Transform T;
    T = Translation(Vector3(0, 0, dhParams.d)) * Eigen::AngleAxisd(dhParams.theta + q, Eigen::Vector3d::UnitZ()) *
        Translation(Vector3(dhParams.a, 0, 0)) * Eigen::AngleAxisd(dhParams.alpha, Eigen::Vector3d::UnitX());

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
    Transform AsJoint = jointTrafos.front();

    AsJoint = m_pose * m_baseOffset * jointTrafos[0];
    AsLink[0] = m_pose * m_baseOffset * Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ()) * m_linkOffsets[0];
    for (size_t i = 1; i < jointTrafos.size(); ++i) {
        AsLink[i] = AsJoint * Eigen::AngleAxisd(angles[i], Eigen::Vector3d::UnitZ()) * m_linkOffsets[i];
        AsJoint = AsJoint * jointTrafos[i];
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
    Transform robotToTcp = this->m_pose;
    for (const auto &trafo : trafos)
        robotToTcp = robotToTcp * trafo;

    return robotToTcp;
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
*  \brief      Set the base offset (rotation and translation) of the base model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  configuration of the base offset
*  \date       2017-11-17
*/
void SerialRobot::setLinkOffsets(const std::vector<Vector6> &linkOffsets) {
    if (linkOffsets.empty())
        return;

    std::vector<Transform> transforms;
    transforms.reserve(linkOffsets.size());
    for (const auto &offset : linkOffsets)
        transforms.push_back(util::poseVecToTransform(offset));
    setLinkOffsets(transforms);
}

/*!
*  \brief      Set the link offsets (Transform) from the dh parameter joint position to the link position.
*  \author     Sascha Kaden
*  \param[in]  Transforms of the link offsets
*  \date       2017-11-17
*/
void SerialRobot::setLinkOffsets(const std::vector<Transform> &linkOffsets) {
    if (linkOffsets.empty())
        return;

    m_linkOffsets = linkOffsets;
}

/*!
*  \brief      Return the link offsets (Transform) from the dh parameter joint position to the link position.
*  \author     Sascha Kaden
*  \param[out] Transform of the link offsets
*  \date       2017-11-17
*/
std::vector<Transform> SerialRobot::getLinkOffsets() const {
    return m_linkOffsets;
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
    if (this->m_baseModel != nullptr) {
        std::vector<Vector3> vertices = this->m_baseModel->m_mesh.vertices;
        cad::transformVertices(m_pose, vertices);
        cad::exportCad(cad::ExportFormat::OBJ, "base", vertices, this->m_baseModel->m_mesh.faces);
    }

    auto linkTrafos = getLinkTrafos(angles);
    for (size_t i = 0; i < m_joints.size(); ++i) {
        Mesh mesh = getModelFromJoint(i)->m_mesh;
        std::vector<Vector3> vertices = mesh.vertices;
        cad::transformVertices(linkTrafos[i], vertices);
        cad::exportCad(cad::ExportFormat::OBJ, "link" + std::to_string(i), vertices, mesh.faces);
    }
}

} /* namespace ippp */
