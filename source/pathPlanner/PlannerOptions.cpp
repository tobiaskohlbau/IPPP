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

#include <core/Logging.h>
#include <pathPlanner/PlannerOptions.h>

using namespace rmpl;

/*!
*  \brief      Standard constructor of the class PlannerOptions
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplingMethod
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
PlannerOptions::PlannerOptions(float trajectoryStepSize,  SamplingMethod method, SamplingStrategy strategy)
    : ModuleBase("PlannerOptions") {
    if (trajectoryStepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set to 1", this);
        m_trajectoryStepSize = 1;
    } else {
        m_trajectoryStepSize = trajectoryStepSize;
    }
    m_samplingStrategy = strategy;
    m_samplingMethod = method;
}

/*!
*  \brief      Sets the trajectory step size
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void PlannerOptions::setTrajectoryStepSize(float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set to 1", this);
        m_trajectoryStepSize = 1;
    } else {
        m_trajectoryStepSize = stepSize;
    }
}

/*!
*  \brief      Returns the trajectory step size
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
float PlannerOptions::getTrajectoryStepSize() {
    return m_trajectoryStepSize;
}

/*!
*  \brief      Sets the sampling method
*  \param[in]  method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void PlannerOptions::setSamplingMethod(SamplingMethod method) {
    m_samplingMethod = method;
}

/*!
*  \brief      Returns the sampling method
*  \param[out] method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
SamplingMethod PlannerOptions::getSamplingMethod() {
    return m_samplingMethod;
}

/*!
*  \brief      Sets the sampling strategy
*  \param[in]  strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
void PlannerOptions::setSamplingStrategy(SamplingStrategy strategy) {
    m_samplingStrategy = strategy;
}

/*!
*  \brief      Returns the strategy method
*  \param[out] strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
SamplingStrategy PlannerOptions::getSamplingStrategy() {
    return m_samplingStrategy;
}
