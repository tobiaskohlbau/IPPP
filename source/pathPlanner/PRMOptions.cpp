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

/*!
*  \brief      Standard constructor of the class PRMOptions
*  \param[in]  rangeSize
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplingMethod
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
PRMOptions::PRMOptions(float rangeSize, float trajectoryStepSize, SamplingMethod method, SamplingStrategy strategy)
    : PlannerOptions(trajectoryStepSize, method, strategy) {
    if (rangeSize <= 0) {
        Logging::warning("Step size was equal or smaller than 0 and is set to 1", "PRM options");
        m_rangeSize = 1;
    } else {
        m_rangeSize = rangeSize;
    }
}

/*!
*  \brief      Sets the range size of the local planner from the PRMPlanner
*  \param[in]  rangeSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void PRMOptions::setRangeSize(float rangeSize) {
    if (rangeSize <= 0) {
        Logging::warning("Step size was equal or smaller than 0 and is set to 1", "PRM options");
        m_rangeSize = 1;
    } else {
        m_rangeSize = rangeSize;
    }
}

/*!
*  \brief      Returns the range size of the local planner from the PRMPlanner
*  \param[out] rangeSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
float PRMOptions::getRangeSize() const {
    return m_rangeSize;
}
