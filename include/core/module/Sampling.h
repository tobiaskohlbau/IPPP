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

#ifndef SAMPLING_H_
#define SAMPLING_H_

#include <math.h>

#include <core/dataObj/Vec.hpp>
#include <core/module/CollisionDetection.h>
#include <core/module/ModuleBase.h>
#include <core/module/Sampler.h>
#include <core/module/TrajectoryPlanner.h>
#include <robot/RobotBase.h>

namespace rmpl {

enum SamplingStrategy { normal, nearObstacles };

/*!
* \brief   Class Sampling creates sample vecs with the passed strategy, for the methods will be used Sampler
* \author  Sascha Kaden
* \date    2016-12-20
*/
class Sampling : public ModuleBase {
  public:
    Sampling(const std::shared_ptr<RobotBase> &robot, const std::shared_ptr<CollisionDetection> &collision,
             const std::shared_ptr<TrajectoryPlanner> &planner, SamplingMethod method = SamplingMethod::randomly,
             SamplingStrategy strategy = SamplingStrategy::normal);

    Vec<float> getSample();
    bool setMeanOfDistribution(const Vec<float> &mean);

  private:
    Vec<float> sampleNearObstacle();

    SamplingStrategy m_strategy;

    std::shared_ptr<CollisionDetection> m_collision;
    std::shared_ptr<RobotBase> m_robot;
    std::shared_ptr<Sampler> m_sampler;
    std::shared_ptr<TrajectoryPlanner> m_planner;
};

} /* namespace rmpl */

#endif /* SAMPLING_H_ */
