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

#include <pathPlanner/PlannerOptions.h>
#include <core/Logging.h>

using namespace rmpl;

PlannerOptions::PlannerOptions(float trajectoryStepSize, TrajectoryMethod trajectoryMethod,
                               SamplingMethod samplingMethod) : Base("Planner options"){
    if (trajectoryStepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set to 1", this);
        m_trajectoryStepSize = 1;
    }
    else {
        m_trajectoryStepSize = trajectoryStepSize;
    }
    m_trajectoryMethod = trajectoryMethod;
    m_samplingMethod = samplingMethod;
}

void PlannerOptions::setTrajectoryStepSize(float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set to 1", this);
        m_trajectoryStepSize = 1;
    }
    else {
        m_trajectoryStepSize = stepSize;
    }
}

float PlannerOptions::getTrajectoryStepSize() {
    return m_trajectoryStepSize;
}

void PlannerOptions::setTrajectoryMethod(TrajectoryMethod method) {
    m_trajectoryMethod = method;
}

TrajectoryMethod PlannerOptions::getTrajectoryMethod() {
    return m_trajectoryMethod;
}

void PlannerOptions::setSamplingMethod(SamplingMethod method) {
    m_samplingMethod = method;
}

SamplingMethod PlannerOptions::getSamplingMethod() {
    return m_samplingMethod;
}
