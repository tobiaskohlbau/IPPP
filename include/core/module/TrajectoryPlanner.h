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

#ifndef TRAJECTORYPLANNER_H_
#define TRAJECTORYPLANNER_H_

#include <core/module/CollisionDetection.h>
#include <core/module/ModuleBase.h>

namespace rmpl {

/*!
* \brief   Class TrajectoryPlanner plans between the passed nodes/vecs. Start and end point aren't part of the path.
* \author  Sascha Kaden
* \date    2016-05-25
*/
class TrajectoryPlanner : public ModuleBase {
  public:
    TrajectoryPlanner(float stepSize, const std::shared_ptr<CollisionDetection> &collision);

    bool controlTrajectory(const Node &source, const Node &target);
    bool controlTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target);
    bool controlTrajectory(const Eigen::VectorXf &source, const Eigen::VectorXf &target);
    std::vector<Eigen::VectorXf> calcTrajectoryCont(const Eigen::VectorXf &source, const Eigen::VectorXf &target);
    std::vector<Eigen::VectorXf> calcTrajectoryBin(const Eigen::VectorXf &source, const Eigen::VectorXf &target);

    void setStepSize(float stepSize);
    float getStepSize() const;

  private:
    float m_stepSize;
    float m_sqStepSize;
    std::shared_ptr<CollisionDetection> m_collision;
};

} /* namespace rmpl */

#endif /* TRAJECTORYPLANNER_H_ */
