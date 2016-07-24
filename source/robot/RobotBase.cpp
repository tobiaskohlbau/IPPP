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

#include <robot/RobotBase.h>

#include <cmath>

#include <Eigen/Geometry>

using namespace rmpl;
using std::shared_ptr;

/*!
*  \brief      Constructor of the class RobotBase
*  \author     Sascha Kaden
*  \param[in]  name of the robot
*  \param[in]  type of the robot
*  \param[in]  dimensions of the robot
*  \param[in]  number of joints of the robot
*  \date       2016-06-30
*/
RobotBase::RobotBase(std::string name, CollisionType type, unsigned int dim, unsigned int numberJoints)
    : Base(name) {
    m_robotName = name;
    m_collisionType = type;
    m_dim = dim;
    m_nbJoints = numberJoints;

    m_pose = Vec<float>(0,0,0,0,0,0);

    m_fileLoader = std::shared_ptr<CadFileLoader>(new CadFileLoader());
}

/*!
*  \brief      Create transformation matrix from the given D-H parameter and the joint angle
*  \author     Sascha Kaden
*  \param[in]  D-H alpha parameter
*  \param[in]  D-H a parameter
*  \param[in]  D-H d parameter
*  \param[in]  joint angle
*  \param[out] transformation matrix
*  \date       2016-07-07
*/
Eigen::Matrix4f RobotBase::getTrafo(float alpha, float a, float d, float q) {
    float sinAlpha = sin(alpha);
    float cosAlpha = cos(alpha);
    float sinQ = sin(q);
    float cosQ = cos(q);

    Eigen::Matrix4f T = Eigen::Matrix4f::Zero(4,4);
    T(0,0) = cosQ;
    T(0,1) = -sinQ * cosAlpha;
    T(0,2) = sinQ * sinAlpha;
    T(0,3) = a * cosQ;
    T(1,0) = sinQ;
    T(1,1) = cosQ * cosAlpha;
    T(1,2) = -cosQ * sinAlpha;
    T(1,3) = a * sinQ;
    T(2,1) = sinAlpha;
    T(2,2) = cosAlpha;
    T(2,3) = d;
    T(3,3) = 1;
    return T;
}

/*!
*  \brief      Compute TCP pose from transformation matrizes and the basis pose
*  \author     Sascha Kaden
*  \param[in]  transformation matrizes
*  \param[in]  basis pose
*  \param[out] TCP pose
*  \date       2016-07-07
*/
Vec<float> RobotBase::getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<float> basis) {
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(basis[3], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(basis[4], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(basis[5], Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f basisToRobot = Eigen::Matrix4f::Zero(4,4);
    basisToRobot.block<3,3>(0,0) = R;
    for (int i = 0; i < 3; ++i)
        basisToRobot(i,3) = basis[i];
    basisToRobot(3,3) = 1;

    // multiply these matrizes together, to get the complete transformation
    // T = A1 * A2 * A3 * A4 * A5 * A6
    Eigen::Matrix4f robotToTcp = trafos[0];
    for (int i = 1; i < 6; ++i)
        robotToTcp *= trafos[i];

    Eigen::Matrix4f basisToTcp = basisToRobot * robotToTcp;

    // create tcp position and orientation vector
    Vec<float> tcp(basisToTcp(0,3), basisToTcp(1,3), basisToTcp(2,3));
    Eigen::Vector3f euler = basisToTcp.block<3,3>(0,0).eulerAngles(0, 1, 2);
    tcp.append(EigenToVec(euler));

    return tcp;
}

/*!
*  \brief      Set boundaries of the robot
*  \author     Sascha Kaden
*  \param[in]  minimum Boudaries
*  \param[in]  maximum Boudaries
*  \date       2016-07-15
*/
void RobotBase::setBoundaries(const Vec<float> &minBoundary, const Vec<float> &maxBoundary) {
    if (minBoundary.empty() || maxBoundary.empty()) {
        this->sendMessage("Boundaries are empty", Message::warning);
        return;
    }
    else if (minBoundary.getDim() != m_dim || maxBoundary.getDim() != m_dim) {
        this->sendMessage("Boudaries have different dimensions from the robot!", Message::warning);
        return;
    }

    for (unsigned int i = 0; i < m_dim; ++i) {
        if (minBoundary[i] > maxBoundary[i]) {
            this->sendMessage("Min boundary is larger than max boundary!", Message::warning);
            return;
        }
    }

    m_maxBoundary = maxBoundary;
    m_minBoundary = minBoundary;
}

/*!
*  \brief      Get minimum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] minimum Boudaries
*  \date       2016-07-15
*/
Vec<float> RobotBase::getMinBoundary() {
    return m_minBoundary;
}

/*!
*  \brief      Get maximum boundary of the robot
*  \author     Sascha Kaden
*  \param[out] maximum Boudaries
*  \date       2016-07-15
*/
Vec<float> RobotBase::getMaxBoundary() {
    return m_maxBoundary;
}

/*!
*  \brief      Set pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[in]  pose Vec
*  \date       2016-07-24
*/
void RobotBase::setPose(const Vec<float> &pose) {
    if (pose.empty()) {
        this->sendMessage("Empty pose vector!", Message::warning);
        return;
    }
    else if (pose.getDim() != 6) {
        this->sendMessage("Pose vector has wrong dimension, must have 6!", Message::warning);
        return;
    }

    m_pose = pose;
}

/*!
*  \brief      Get pose of robot, translation and rotation (x,y,z,rx,ry,rz)
*  \author     Sascha Kaden
*  \param[out] pose Vec
*  \date       2016-07-24
*/
Vec<float> RobotBase::getPose() {
    return m_pose;
}

/*!
*  \brief      Load cad models from given string vector and save them intern
*  \author     Sascha Kaden
*  \param[in]  vector of file strings
*  \param[out] true if loading was feasible
*  \date       2016-06-30
*/
bool RobotBase::setCadModels(const std::vector<std::string> &files) {
    m_cadFiles = files;
    m_cadModels.clear();

    std::shared_ptr<PQP_Model> model;
    for (auto file : files) {
        model = m_fileLoader->loadFile(file);
        if (model == nullptr)
            return false;
        else
            m_cadModels.push_back(model);
    }

    return true;
}

/*!
*  \brief      Return PQP cad model from index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] PQP cad model
*  \date       2016-06-30
*/
std::shared_ptr<PQP_Model> RobotBase::getCadModel(unsigned int index) {
    if (index >= m_cadModels.size()) {
        this->sendMessage("model index is larger than cad models");
        return nullptr;
    }
    else {
        return m_cadModels[index];
    }
}

/*!
*  \brief      Set workspace to robot
*  \author     Sascha Kaden
*  \param[in]  file of workspace cad
*  \date       2016-07-14
*/
bool RobotBase::setWorkspace(const std::string &workspaceFile) {
    shared_ptr<PQP_Model> model = m_fileLoader->loadFile(workspaceFile);
    if (model == nullptr)
        return false;

    m_workspaceFile = workspaceFile;
    m_workspaceCad = model;
    return true;
}

/*!
*  \brief      Return workspace of robot
*  \author     Sascha Kaden
*  \param[out] pointer to PQP_Model
*  \date       2016-07-14
*/
shared_ptr<PQP_Model> RobotBase::getWorkspace() {
    if (m_workspaceCad != nullptr) {
        this->sendMessage("workspace is not set!", Message::error);
        return m_workspaceCad;
    }
    else {
        return nullptr;
    }
}

/*!
*  \brief      Set 2D workspace to robot
*  \author     Sascha Kaden
*  \param[in]  2D workspace
*  \date       2016-07-14
*/
bool RobotBase::set2DWorkspace(const Eigen::MatrixXi &workspace) {
    m_2DWorkspace = workspace;
    return true;
}

/*!
*  \brief      Return 2D workspace of robot
*  \author     Sascha Kaden
*  \param[out] 2D workspace
*  \date       2016-07-14
*/
Eigen::MatrixXi& RobotBase::get2DWorkspace() {
    return m_2DWorkspace;
}

/*!
*  \brief      Return dimension from the robot
*  \author     Sascha Kaden
*  \param[out] dimension
*  \date       2016-06-30
*/
unsigned int RobotBase::getDim() {
    return m_dim;
}

/*!
*  \brief      Return number of the joints from the robot
*  \author     Sascha Kaden
*  \param[out] number of joints
*  \date       2016-06-30
*/
unsigned int RobotBase::getNbJoints() {
    return m_nbJoints;
}

/*!
*  \brief      Return name from the robot
*  \author     Sascha Kaden
*  \param[out] name
*  \date       2016-06-30
*/
std::string RobotBase::getName() {
    return m_robotName;
}

/*!
*  \brief      Return the collision type of the robot
*  \author     Sascha Kaden
*  \param[out] CollisionType
*  \date       2016-06-30
*/
CollisionType RobotBase::getCollisionType() {
    return m_collisionType;
}

/*!
*  \brief      Convert deg angles to rad
*  \author     Sascha Kaden
*  \param[in]  Vec of deg angles
*  \param[out] Vec of rad angles
*  \date       2016-07-07
*/
Vec<float> RobotBase::degToRad(const Vec<float> deg) {
    Vec<float> rad(m_dim);
    for (unsigned int i = 0; i < m_dim; ++i)
        rad[i] = deg[i] / 360 * m_pi;
    return rad;
}

/*!
*  \brief      Convert rmpl Vec to Eigen Array
*  \author     Sascha Kaden
*  \param[in]  Vec
*  \param[out] Eigen Array
*  \date       2016-07-07
*/
Eigen::ArrayXf RobotBase::VecToEigen(const Vec<float> &vec) {
    Eigen::ArrayXf eigenVec(vec.getDim());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        eigenVec(i, 0) = vec[i];
    return eigenVec;
}

/*!
*  \brief      Convert Eigen Array to rmpl Vec
*  \author     Sascha Kaden
*  \param[in]  Eigen Array
*  \param[out] Vec
*  \date       2016-07-07
*/
Vec<float> RobotBase::EigenToVec(const Eigen::ArrayXf &eigenVec) {
    Vec<float> vec((unsigned int)eigenVec.rows());
    for (unsigned int i = 0; i < vec.getDim(); ++i)
        vec[i] = eigenVec(i, 0);
    return vec;
}
