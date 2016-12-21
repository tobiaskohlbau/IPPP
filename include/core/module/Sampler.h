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

#ifndef SAMPLER_H_
#define SAMPLER_H_

#include <math.h>
#include <random>
#include <stdlib.h>
#include <time.h>

#include <core/module/ModuleBase.h>
#include <core/dataObj/Vec.hpp>
#include <robot/RobotBase.h>

namespace rmpl {

enum SamplingMethod { randomly, uniform, standardDistribution };

/*!
* \brief   Class Sampling creates sample vecs with the passed method
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Sampler : public ModuleBase {
  public:
    Sampler(const std::shared_ptr<RobotBase> &robot, SamplingMethod method = SamplingMethod::randomly);
    Vec<float> getSample();
    float getRandomAngle();

    bool setMeanOfDistribution(const Vec<float> &mean);

  private:
    bool checkBoudaries();
    Vec<float> sampleStandardDist();
    Vec<float> sampleUniform();
    Vec<float> sampleRandom();

    unsigned int m_dim;
    Vec<float> m_minBoundary;
    Vec<float> m_maxBoundary;
    SamplingMethod m_method;
    std::shared_ptr<RobotBase> m_robot;

    std::random_device rd;
    std::mt19937 m_generator;
    std::vector<std::normal_distribution<float>> m_distNormal;
    std::vector<std::uniform_real_distribution<float>> m_distUniform;
    std::uniform_real_distribution<float> m_distAngle;
};

} /* namespace rmpl */

#endif /* SAMPLER_H_ */
