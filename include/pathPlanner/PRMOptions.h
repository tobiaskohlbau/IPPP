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

#ifndef PRMOPTIONS_H_
#define PRMOPTIONS_H_

#include <pathPlanner/PlannerOptions.h>

namespace rmpl{

class PRMOptions : public PlannerOptions {
public:
    PRMOptions(float rangeSize, float trajectoryStepSize, TrajectoryMethod trajectoryMethod, SamplingMethod samplingMethod);

    void setRangeSize(float rangeSize);
    float getRangeSize();

private:
    float m_rangeSize;
};

} /* namespace rmpl */

#endif //PRMOPTIONS_H_
