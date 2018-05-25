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
#include <ippp/util/UtilGeo.hpp>

namespace ippp {

/*!
*  \brief      Constructor of the class SerialRobot
*  \author     Sascha Kaden
*  \param[in]  dimension (number of joints)
*  \param[in]  joint containers
*  \param[in]  DofTypes of the joints
*  \param[in]  name of the robot
*  \date       2018-01-10
*/
SerialRobot::SerialRobot(unsigned int dim, const std::vector<Joint> &joints, const std::vector<DofType> &dofTypes,
                         const std::string &name)
    : RobotBase(name, dim, RobotCategory::serial, dofTypes),
      m_baseOffset(Transform::Identity()),
      m_tcpOffset(Transform::Identity()),
      m_toolModelOffset(Transform::Identity()),
      m_joints(joints) {
    if (joints.size() != dim || dofTypes.size() != dim) {
        Logging::error("Dimension to parameter sizes is unequal", this);
        return;
    }
    updateJointParams();
}

/*!
*  \brief      Calculate the Transform of the TCP of the robot.
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] TCP transformation
*  \date       2017-06-21
*/
Transform SerialRobot::getTransformation(const VectorX &config) const {
    return getTcp(this->getJointTrafos(config));
}

/*!
*  \brief      Get vector of Jaco transformation matrices
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of transformation matrices
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
*  \param[in]  D-H parameter
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
*  \brief      Get vector of Jaco transformation matrices
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
        AsLink[i] = AsJoint * m_linkOffsets[i] * Eigen::AngleAxisd(angles[i], Eigen::Vector3d::UnitZ());
        AsJoint = AsJoint * jointTrafos[i];
    }

    return AsLink;
}

/*!
*  \brief      Get vector of Jaco transformation matrices
*  \author     Sascha Kaden
*  \param[in]  real angles
*  \param[out] vector of Transforms
*  \date       2016-10-22
*/
std::pair<std::vector<Transform>, Transform> SerialRobot::getLinkAndToolTrafos(const VectorX &angles) const {
    std::vector<Transform> jointTrafos = getJointTrafos(angles);
    std::vector<Transform> AsLink(jointTrafos.size());
    Transform AsJoint = jointTrafos.front();

    AsJoint = m_pose * m_baseOffset * jointTrafos[0];
    AsLink[0] = m_pose * m_baseOffset * Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitZ()) * m_linkOffsets[0];
    for (size_t i = 1; i < jointTrafos.size(); ++i) {
        AsLink[i] = AsJoint * Eigen::AngleAxisd(angles[i], Eigen::Vector3d::UnitZ()) * m_linkOffsets[i];
        AsJoint = AsJoint * jointTrafos[i];
    }

    return std::make_pair(AsLink, AsJoint * m_toolModelOffset);
}

/*!
*  \brief      Compute tool center pose from transformation matrices and the basis pose
*  \author     Sascha Kaden
*  \param[in]  transformation matrices
*  \param[out] TCP pose as Transform
*  \date       2016-07-07
*/
Transform SerialRobot::getTcp(const std::vector<Transform> &jointTrafos) const {
    // multiply these matrices together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Transform robotToTcp = m_pose * m_baseOffset;
    for (const auto &trafo : jointTrafos)
        robotToTcp = robotToTcp * trafo;

    return robotToTcp * m_tcpOffset;
}

/*!
*  \brief      Calculate the jacobian matrix to the passed configuration.
*  \detail     Calculation of the jacobian with the vector cross product method. The method was taken from Robotics - control,
* sensing, vision and intelligence from Fu, Gonzalez and Lee.
*  \author     Sascha Kaden
*  \param[in]  configuration of the base offset
*  \date       2017-11-17
*/
MatrixX SerialRobot::calcJacobian(const VectorX &config) const {
    MatrixX jacobi(6, m_dim);
#if 1
    std::vector<Transform> jointTrafos = getJointTrafos(config);
    Transform tcp = getTcp(jointTrafos);

    Transform frame = m_pose * m_baseOffset;
    for (size_t i = 0; i < m_dim; ++i) {
        if (i > 0)
            frame = frame * jointTrafos[i - 1];

        jacobi.block<3, 1>(0, i) = m_zUnitVectors[i].cross(tcp.translation() - frame.translation());
        jacobi.block<3, 1>(3, i) = m_zUnitVectors[i];
    }
#else
    Vector6 tcp = util::transformToVec(getTcp(getJointTrafos(config)));

    const double diff = 0.0001;
    for (size_t i = 0; i < m_dim; ++i) {
        auto tmpConfig = config;
        tmpConfig[i] += diff;
        Vector6 tmpTcp = util::transformToVec(getTcp(getJointTrafos(tmpConfig)));
        jacobi.block<6, 1>(0, i) = (tmpTcp - tcp) / diff;
    }
#endif
    return jacobi;
}

/*!
*  \brief      Set the base offset (rotation and translation) of the base model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  configuration of the base offset
*  \date       2017-11-17
*/
void SerialRobot::setBaseOffset(const Vector6 &baseOffset) {
    setBaseOffset(util::toTransform(baseOffset));
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
*  \brief      Set the tool offset (rotation and translation) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  configuration of the tool offset
*  \date       2018-01-08
*/
void SerialRobot::setTcpOffset(const Vector6 &tcpOffset) {
    setTcpOffset(util::toTransform(tcpOffset));
}

/*!
*  \brief      Set the tool offset (Transform) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  Transform of the tool offset
*  \date       2018-01-08
*/
void SerialRobot::setTcpOffset(const Transform &tcpOffset) {
    m_tcpOffset = tcpOffset;
}

/*!
*  \brief      Return the tool offset (Transform) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[out] Transform of the tool offset
*  \date       2018-01-08
*/
Transform SerialRobot::getTcpOffset() const {
    return m_tcpOffset;
}

/*!
*  \brief      Set the tool offset (rotation and translation) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  configuration of the tool offset
*  \date       2018-01-08
*/
void SerialRobot::setToolModelOffset(const Vector6 &toolOffset) {
    setToolModelOffset(util::toTransform(toolOffset));
}

/*!
*  \brief      Set the tool offset (Transform) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[in]  Transform of the tool offset
*  \date       2018-01-08
*/
void SerialRobot::setToolModelOffset(const Transform &toolOffset) {
    m_toolModelOffset = toolOffset;
}

/*!
*  \brief      Return the tool offset (Transform) of the tool model of the serial robot.
*  \author     Sascha Kaden
*  \param[out] Transform of the tool offset
*  \date       2018-01-08
*/
Transform SerialRobot::getToolModelOffset() const {
    return m_toolModelOffset;
}

/*!
*  \brief      Set the tool model of the robot.
*  \author     Sascha Kaden
*  \param[in]  tool model
*  \date       2018-01-08
*/
void SerialRobot::setToolModel(const std::shared_ptr<ModelContainer> &model) {
    if (!model || model->empty()) {
        Logging::error("Empty tool model", this);
        return;
    }
    m_toolModel = model;
}

/*!
*  \brief      Returns the tool model of the robot.
*  \author     Sascha Kaden
*  \param[out] tool model
*  \date       2018-01-08
*/
std::shared_ptr<ModelContainer> SerialRobot::getToolModel() const {
    return m_toolModel;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
size_t SerialRobot::numJoints() const {
    return m_joints.size();
}

/*!
*  \brief      Return ModelContainer from link by index
*  \author     Sascha Kaden
*  \param[in]  link index
*  \param[out] shared_ptr of the ModelContainer
*  \date       2018-01-10
*/
std::shared_ptr<ModelContainer> SerialRobot::getLinkModel(size_t index) const {
    if (index >= m_linkModels.size()) {
        Logging::error("Joint index larger than joint size", this);
        return nullptr;
    }
    return m_linkModels[index];
}

/*!
*  \brief      Return ModelContainer vector from all links
*  \author     Sascha Kaden
*  \param[out] vector of ModelContainer ptr
*  \date       2018-01-10
*/
std::vector<std::shared_ptr<ModelContainer>> SerialRobot::getLinkModels() const {
    return m_linkModels;
}

/*!
*  \brief      Return the link offsets (Transform) from the dh parameter joint position to the link position.
*  \author     Sascha Kaden
*  \param[out] Transform of the link offsets
*  \date       2018-01-10
*/
std::vector<Transform> SerialRobot::getLinkOffsets() const {
    return m_linkOffsets;
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  joint angles
*  \date       2018-01-10
*/
void SerialRobot::saveMeshConfig(const VectorX &angles) {
    auto linkTrafos = getLinkTrafos(angles);

    if (m_baseModel != nullptr) {
        std::vector<Vector3> vertices = m_baseModel->m_mesh.vertices;
        cad::transformVertices(m_pose, vertices);
        cad::exportCad(cad::ExportFormat::OBJ, "base", vertices, m_baseModel->m_mesh.faces);
    }

    for (size_t i = 0; i < m_joints.size(); ++i) {
        std::vector<Vector3> vertices = m_linkModels[i]->m_mesh.vertices;
        cad::transformVertices(linkTrafos[i], vertices);
        cad::exportCad(cad::ExportFormat::OBJ, "link" + std::to_string(i), vertices, m_linkModels[i]->m_mesh.faces);
    }

    if (m_toolModel != nullptr) {
        auto tcp = getTransformation(angles);
        std::vector<Vector3> vertices = m_toolModel->m_mesh.vertices;
        cad::transformVertices(tcp, vertices);
        cad::exportCad(cad::ExportFormat::OBJ, "tool", vertices, m_toolModel->m_mesh.faces);
    }
}

/*!
*  \brief      Update all parameter of the joints into container of the SerialRobot class for a better performance.
*  \author     Sascha Kaden
*  \date       2018-01-10
*/
void SerialRobot::updateJointParams() {
    m_linkOffsets.clear();
    m_dhParameters.clear();
    m_linkModels.clear();
    m_linkOffsets.reserve(m_joints.size());
    m_dhParameters.reserve(m_joints.size());
    m_linkModels.reserve(m_joints.size());

    for (auto &joint : m_joints) {
        m_linkOffsets.push_back(joint.getLinkOffset());
        m_dhParameters.push_back(joint.getDhParameter());
        m_linkModels.push_back(joint.getLinkModel());
    }

    m_boundary.first = VectorX(m_dim);
    m_boundary.second = VectorX(m_dim);
    for (size_t i = 0; i < m_dim; ++i) {
        auto boundaries = m_joints[i].getBoundaries();
        m_boundary.first[i] = boundaries.first;
        m_boundary.second[i] = boundaries.second;
    }

    Vector3 zUnit(0, 0, 1);
    zUnit = m_baseOffset.rotation() * m_pose.rotation() * zUnit;
    m_zUnitVectors.clear();
    m_zUnitVectors.push_back(zUnit);
    for (size_t i = 1; i < m_dim; ++i)
        m_zUnitVectors.push_back(Eigen::AngleAxisd(m_dhParameters[i - 1].alpha, Eigen::Vector3d::UnitX()) *
                                 m_zUnitVectors[i - 1]);
}

} /* namespace ippp */
