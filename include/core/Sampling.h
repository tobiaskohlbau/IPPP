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
#include <stdlib.h>
#include <time.h>

#include <core/Base.h>
#include <core/Vec.hpp>
#include <robot/RobotBase.h>

namespace rmpl {

enum SamplingMethod { randomly, uniform, hammersley, halton };

/*!
* \brief   Class Sampling creates sample vecs with the passed method
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Sampling : public Base {
  public:
    Sampling(const std::shared_ptr<RobotBase> &robot, SamplingMethod method = SamplingMethod::randomly);
    Vec<float> getSample(unsigned int dim, int index, int nbSamples);

  private:
    bool checkBoudaries();

    Vec<float> m_minBoundary;
    Vec<float> m_maxBoundary;
    SamplingMethod m_method;
    std::shared_ptr<RobotBase> m_robot;
};

} /* namespace rmpl */

#endif /* SAMPLING_H_ */
