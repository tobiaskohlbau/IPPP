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

#include <pathPlanner/PRMOptions.h>

#include <core/Logging.h>

using namespace rmpl;

PRMOptions::PRMOptions(float rangeSize, float trajectoryStepSize, TrajectoryMethod trajectoryMethod,
                       SamplingMethod samplingMethod) : PlannerOptions(trajectoryStepSize, trajectoryMethod, samplingMethod) {
    if (rangeSize <= 0) {
        Logging::warning("Step size was smaller than 0 and was set to 1", this);
        m_rangeSize = 1;
    }
    else {
        m_rangeSize = rangeSize;
    }
}

void PRMOptions::setRangeSize(float rangeSize) {
    if (rangeSize <= 0) {
        Logging::warning("Step size was smaller than 0 and was set to 1", this);
        m_rangeSize = 1;
    }
    else {
        m_rangeSize = rangeSize;
    }
}

float PRMOptions::getRangeSize() {
    return m_rangeSize;
}
