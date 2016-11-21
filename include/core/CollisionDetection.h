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

#ifndef COLLISIONDETECTION_H_
#define COLLISIONDETECTION_H_

#include <Eigen/Core>
#include <PQP.h>
#include <fcl/fcl.h>
#include <fcl/narrowphase/collision.h>

#include <core/ModuleBase.h>
#include <core/Node.h>
#include <core/Triangle.h>
#include <robot/MeshContainer.h>
#include <robot/SerialRobot.h>
#include <robot/TriangleRobot2D.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return binary value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetection : public ModuleBase {
  public:
    CollisionDetection(const std::shared_ptr<RobotBase> &robot);
    bool controlVec(const Vec<float> &vec);
    bool controlTrajectory(std::vector<Vec<float>> &vec);

  private:
    bool controlCollisionMesh(const Vec<float> &vec);
    bool checkSerialRobot(const Vec<float> &vec);
    bool checkMobileRobot(const Vec<float> &vec);
    bool checkMesh(std::vector<std::shared_ptr<PQP_Model>> &models, std::shared_ptr<PQP_Model> &base, Eigen::Matrix3f R[],
                   Eigen::Matrix3f &poseR, Eigen::Vector3f t[], Eigen::Vector3f &poseT);
    bool checkMesh(std::vector<std::shared_ptr<FCLModel>> &models, std::shared_ptr<FCLModel> &base, Eigen::Matrix3f R[],
                   Eigen::Matrix3f &poseR, Eigen::Vector3f t[], Eigen::Vector3f &poseT);

    bool checkPQP(std::shared_ptr<PQP_Model> &model1, std::shared_ptr<PQP_Model> &model2, Eigen::Matrix3f &R1,
                  Eigen::Matrix3f &R2, Eigen::Vector3f &t1, Eigen::Vector3f &t2);
    bool checkFCL(std::shared_ptr<FCLModel> &model1, std::shared_ptr<FCLModel> &model2, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                  Eigen::Vector3f &t1, Eigen::Vector3f &t2);

    bool checkPoint2D(float x, float y);
    bool checkTriangleRobot(const Vec<float> &vec);

    std::shared_ptr<RobotBase> m_robot;
    Vec<float> m_minBoundary, m_maxBoundary;

    Eigen::MatrixXi m_2DWorkspace;
    std::shared_ptr<MeshContainer> m_workspace = nullptr;

    // models for collision detection
    fcl::CollisionObject<float> *o1;
    fcl::CollisionObject<float> *o2;

    Eigen::Matrix3f m_identity;
    Eigen::Vector3f m_zeroVec;
};

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
