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

#include <core/ModuleBase.h>
#include <core/Sampling.h>
#include <core/TrajectoryPlanner.h>

namespace rmpl {

/*!
* \brief   Class PlannerOptions determines all base options for the path planner
* \author  Sascha Kaden
* \date    2016-08-29
*/
class PlannerOptions {
  public:
    PlannerOptions(float trajectoryStepSize, SamplingMethod samplingMethod, SamplingStrategy strategy);

    void setTrajectoryStepSize(float stepSize);
    float getTrajectoryStepSize() const;
    void setSamplingMethod(SamplingMethod method);
    SamplingMethod getSamplingMethod() const;
    void setSamplingStrategy(SamplingStrategy strategy);
    SamplingStrategy getSamplingStrategy() const;

  protected:
    float m_trajectoryStepSize;
    SamplingMethod m_samplingMethod;
    SamplingStrategy m_samplingStrategy;
};

} /* namespace rmpl */

#endif    // PLANNEROPTIONS_H_
