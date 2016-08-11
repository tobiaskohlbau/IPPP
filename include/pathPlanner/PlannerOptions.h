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

#ifndef PLANNEROPTIONS_H_
#define PLANNEROPTIONS_H_

#include <core/Base.h>
#include <core/Sampling.h>
#include <core/TrajectoryPlanner.h>

namespace rmpl {

class PlannerOptions : public Base {
  public:
    PlannerOptions(float trajectoryStepSize, TrajectoryMethod trajectoryMethod, SamplingMethod samplingMethod);

    void setTrajectoryStepSize(float stepSize);
    float getTrajectoryStepSize();
    void setTrajectoryMethod(TrajectoryMethod method);
    TrajectoryMethod getTrajectoryMethod();
    void setSamplingMethod(SamplingMethod method);
    SamplingMethod getSamplingMethod();

  protected:
    float m_trajectoryStepSize;
    TrajectoryMethod m_trajectoryMethod;
    SamplingMethod m_samplingMethod;
};

} /* namespace rmpl */

#endif    // PLANNEROPTIONS_H_
