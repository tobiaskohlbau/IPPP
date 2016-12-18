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

#include <include/pathPlanner/options/RRTOptions.h>

#include <core/Logging.h>

using namespace rmpl;

/*!
*  \brief      Standard constructor of the class RRTOptions
*  \param[in]  RRT step size
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplingMethod
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
RRTOptions::RRTOptions(float stepSize, float trajectoryStepSize,  SamplingMethod method,
                       SamplingStrategy strategy) : PlannerOptions(trajectoryStepSize, method, strategy) {
    if (stepSize <= 0) {
        Logging::warning("Step size was smaller than 0 and was set to 1", "RRT options");
        m_stepSize = 1;
    }
    else {
        m_stepSize = stepSize;
    }
}

/*!
*  \brief      Sets the step size of the RRTPlanner
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void RRTOptions::setStepSize(float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Step size was smaller than 0 and was set to 1", "RRT options");
        m_stepSize = 1;
    }
    else {
        m_stepSize = stepSize;
    }
}

/*!
*  \brief      Returns the step size of the RRTPlanner
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
float RRTOptions::getStepSize() const {
    return m_stepSize;
}
