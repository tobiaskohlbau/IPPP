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

#include <core/Base.h>
#include <core/Node.h>
#include <robot/MeshContainer.h>
#include <robot/MobileRobot.h>
#include <robot/RobotBase.h>
#include <robot/SerialRobot.h>

namespace rmpl {

/*!
* \brief   Class CollisionDetection checks the configuration on collision and return binary value
* \author  Sascha Kaden
* \date    2016-05-25
*/
class CollisionDetection : public Base {
  public:
    CollisionDetection(const std::shared_ptr<RobotBase> &robot);
    bool controlCollision(const Vec<float> &vec);
    bool controlCollision(const std::vector<Vec<float>> &vec);

  private:
    bool checkPqpSerialRobot(const Vec<float> &vec);
    bool checkPqpMobileRobot(const Vec<float> &vec);
    bool checkFclSerialRobot(const Vec<float> &vec);
    bool checkFclMobileRobot(const Vec<float> &vec);

    bool controlCollisionPqp(const Vec<float> &vec);
    bool controlCollisionFcl(const Vec<float> &vec);
    bool checkPQP(std::shared_ptr<PQP_Model> &model1, std::shared_ptr<PQP_Model> &model2, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                  Eigen::Vector3f &t1, Eigen::Vector3f &t2);
    bool checkFcl(std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> &model1,
                  std::shared_ptr<fcl::BVHModel<fcl::OBBRSS<float>>> &model2, Eigen::Matrix3f &R1, Eigen::Matrix3f &R2,
                  Eigen::Vector3f &t1, Eigen::Vector3f &t2);
    bool controlCollisionPointRobot(float x, float y);

    std::shared_ptr<RobotBase> m_robot;

    Eigen::MatrixXi m_2DWorkspace;
    std::shared_ptr<MeshContainer> m_workspace;
};

} /* namespace rmpl */

#endif /* COLLISIONDETECTION_H_ */
