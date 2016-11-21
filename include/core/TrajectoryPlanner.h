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

#include <core/CollisionDetection.h>
#include <core/ModuleBase.h>

namespace rmpl {

enum TrajectoryMethod { linear, spline };

/*!
* \brief   Class TrajectoryPlanner plans between the passed nodes/vecs
* \author  Sascha Kaden
* \date    2016-05-25
*/
class TrajectoryPlanner : public ModuleBase {
  public:
    TrajectoryPlanner(TrajectoryMethod method, float stepSize, const std::shared_ptr<CollisionDetection> &collision);

    bool controlTrajectory(const Node &source, const Node &target);
    bool controlTrajectory(const std::shared_ptr<Node> &source, const std::shared_ptr<Node> &target);
    bool controlTrajectory(const Vec<float> &source, const Vec<float> &target);
    std::vector<Vec<float>> computeTrajectory(const Vec<float> &source, const Vec<float> &target);
    std::vector<Vec<float>> computeTrajectory(const Vec<float> &source, const Vec<float> &target, float stepSize);

    void setStepSize(float stepSize);
    float getStepSize();

  private:
    TrajectoryMethod m_method;
    float m_stepSize;
    std::shared_ptr<CollisionDetection> m_collision;
};

} /* namespace rmpl */

#endif /* TRAJECTORYPLANNER_H_ */
