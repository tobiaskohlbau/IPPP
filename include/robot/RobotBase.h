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

#include <string>
#include <vector>

#include <Eigen/Core>
#include <PQP.h>

#include <core/module/ModuleBase.h>
#include <robot/MeshContainer.h>

namespace rmpl {

enum CollisionType { fcl, pqp, point2D, triangle2D };

enum RobotType { serial, mobile };

/*!
* \brief   Base class of all robots
* \author  Sascha Kaden
* \date    2016-06-30
*/
class RobotBase : public ModuleBase {
  public:
    RobotBase(std::string name, CollisionType collisionType, RobotType robotType, unsigned int dim);

    Eigen::VectorXf getMinBoundary();
    Eigen::VectorXf getMaxBoundary();

    void setPose(const Eigen::Matrix<float, 6, 1> &pose);
    Eigen::Matrix<float, 6, 1> getPose();
    Eigen::Matrix4f getPoseMat();

    void setBaseMesh(const std::shared_ptr<MeshContainer> &baseMesh);
    std::shared_ptr<MeshContainer> getBaseMesh();

    void setWorkspace(const std::shared_ptr<MeshContainer> &mesh);
    std::shared_ptr<MeshContainer> getWorkspace();
    void set2DWorkspace(const Eigen::MatrixXi &space);
    Eigen::MatrixXi &get2DWorkspace();

    unsigned int getDim();
    RobotType getRobotType();

    CollisionType getCollisionType();
    void setCollisionType(CollisionType type);

  protected:
    CollisionType m_collisionType;
    RobotType m_robotType;
    unsigned int m_dim;

    Eigen::VectorXf m_minBoundary;
    Eigen::VectorXf m_maxBoundary;
    Eigen::Matrix<float, 6, 1> m_pose;
    Eigen::Matrix4f m_poseMat;

    std::shared_ptr<MeshContainer> m_baseMesh;
    std::shared_ptr<MeshContainer> m_workspaceMesh;
    Eigen::MatrixXi m_2DWorkspace;
};

} /* namespace rmpl */

#endif /* ROBOTBASE_H_ */
