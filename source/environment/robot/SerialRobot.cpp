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
    : RobotBase(name, dim, RobotType::serial, boundary, dofTypes) {
}

/*!
*  \brief      Compute the transformation of the robot from the configuration
*  \author     Sascha Kaden
*  \param[in]  configuration
*  \param[out] pair with rotation and translation
*  \date       2017-06-21
*/
std::pair<Matrix3, Vector3> SerialRobot::getTransformation(const VectorX &config) const {
    return std::make_pair(Matrix3(), Vector3());
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
Matrix4 SerialRobot::getTrafo(double alpha, double a, double d, double q) {
    double sinAlpha = sin(alpha);
    double cosAlpha = cos(alpha);
    double sinQ = sin(q);
    double cosQ = cos(q);

    Matrix4 T = Matrix4::Zero(4, 4);
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
Vector6 SerialRobot::getTcpPosition(const std::vector<Matrix4> &trafos) {
    // multiply these matrizes together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Matrix4 robotToTcp = trafos[0];
    for (int i = 1; i < 6; ++i) {
        robotToTcp *= trafos[i];
    }
    Matrix4 basisToTcp = this->m_poseMat * robotToTcp;

    return util::poseMatToVec(basisToTcp);
}

/*!
*  \brief      Return MeshContainer from joint by index
*  \author     Sascha Kaden
*  \param[in]  joint index
*  \param[out] MeshContainer
*  \date       2016-08-25
*/
std::shared_ptr<ModelContainer> SerialRobot::getModelFromJoint(unsigned int jointIndex) {
    if (jointIndex < m_joints.size()) {
        return m_joints[jointIndex].getModel();
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
std::vector<std::shared_ptr<ModelContainer>> SerialRobot::getJointModels() {
    std::vector<std::shared_ptr<ModelContainer>> models;
    for (auto joint : m_joints) {
        models.push_back(joint.getModel());
    }
    return models;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
unsigned int SerialRobot::getNbJoints() {
    return m_joints.size();
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  joint angles
*  \date       2016-10-22
*/
void SerialRobot::saveMeshConfig(VectorX angles) {
    std::vector<Matrix4> jointTrafos = getJointTrafos(angles);
    Matrix4 As[jointTrafos.size()];
    As[0] = this->m_poseMat * jointTrafos[0];
    for (int i = 1; i < jointTrafos.size(); ++i) {
        As[i] = As[i - 1] * jointTrafos[i];
    }
    saveMeshConfig(As);
}

/*!
*  \brief      Saves the configuration of the robot by obj files in the working directory
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \date       2016-10-22
*/
void SerialRobot::saveMeshConfig(Matrix4 *As) {
    if (this->m_baseModel != nullptr) {
        std::vector<Vector3> verts;
        for (auto vertice : this->m_baseModel->m_mesh.vertices) {
            Vector4 temp(util::append<3>(vertice, (double)1));
            temp = this->m_poseMat * temp;
            verts.push_back(Vector3(temp(0), temp(1), temp(2)));
        }
        cad::exportCad(cad::ExportFormat::OBJ, "base", verts, this->m_baseModel->m_mesh.faces);
    }
    // this->m_baseModel->saveObj("base.obj", this->m_poseMat);

    for (int i = 0; i < m_joints.size(); ++i) {
        std::vector<Vector3> verts;
        for (auto vertice : getModelFromJoint(i)->m_mesh.vertices) {
            Vector4 temp(util::append<3>(vertice, (double)1));
            temp = this->m_poseMat * temp;
            verts.push_back(Vector3(temp(0), temp(1), temp(2)));
        }
        cad::exportCad(cad::ExportFormat::OBJ, "link" + std::to_string(i), verts, getModelFromJoint(i)->m_mesh.faces);
        // getModelFromJoint(i)->saveObj("link" + std::to_string(i) + ".obj", As[i]);
        // std::cout<< As[i] << std::endl <<std::endl;
    }
}

} /* namespace ippp */
