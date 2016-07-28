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

#ifndef ROBOTBASE_H_
#define ROBOTBASE_H_

#include <vector>
#include <string>

#include <Eigen/Core>
#include <PQP.h>

#include <core/Base.h>
#include <core/Vec.hpp>
#include <robot/CadFileLoader.h>

namespace rmpl {

enum CollisionType
{
    vrep,
    pqp,
    twoD
};

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public Base
{
public:
    RobotBase(std::string name, CollisionType type, unsigned int dim, unsigned int numberJoints);

    virtual Vec<REAL> directKinematic(const Vec<REAL> &angles) = 0;
    virtual std::vector<Eigen::Matrix4f> getTransformations(const Vec<REAL> &angles) = 0;
    Eigen::Matrix4f getTrafo(REAL alpha, REAL a, REAL d, REAL q);
    Vec<REAL> getTcpPosition(const std::vector<Eigen::Matrix4f> &trafos, const Vec<REAL> basis);

    void setBoundaries(const Vec<REAL> &minBoundary, const Vec<REAL> &maxBoundary);
    Vec<REAL> getMinBoundary();
    Vec<REAL> getMaxBoundary();

    void setPose(const Vec<REAL> &pose);
    Vec<REAL> getPose();

    bool setCadModels(const std::vector<std::string> &files);
    std::shared_ptr<PQP_Model> getCadModel(unsigned int index);

    bool setWorkspace(const std::string &workspaceFile);
    std::shared_ptr<PQP_Model> getWorkspace();
    bool set2DWorkspace(const Eigen::MatrixXi &space);
    Eigen::MatrixXi& get2DWorkspace();

    unsigned int getDim();
    unsigned int getNbJoints();
    std::string getName();
    CollisionType getCollisionType();
    void setCollisionType(CollisionType type);

    Vec<REAL> degToRad(const Vec<REAL> deg);
    Eigen::ArrayXf VecToEigen(const Vec<REAL> &vec);
    Vec<REAL> EigenToVec(const Eigen::ArrayXf &eigenVec);

protected:
    std::shared_ptr<CadFileLoader> m_fileLoader;

    std::string   m_robotName;
    CollisionType m_collisionType;
    unsigned int  m_nbJoints;
    unsigned int  m_dim;

    Vec<REAL> m_minBoundary;
    Vec<REAL> m_maxBoundary;
    Vec<REAL> m_alpha;
    Vec<REAL> m_a;
    Vec<REAL> m_d;
    Vec<REAL> m_pose;

    std::vector<std::string> m_cadFiles;
    std::vector<std::shared_ptr<PQP_Model>> m_cadModels;
    std::string m_workspaceFile;
    std::shared_ptr<PQP_Model> m_workspaceCad;
    Eigen::MatrixXi m_2DWorkspace;

    REAL m_pi = 3.1416;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
