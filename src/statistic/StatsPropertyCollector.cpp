//-------------------------------------------------------------------------//
//
// Copyright 2017 Sascha Kaden
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

#include <ippp/statistic/StatsPropertyCollector.h>

namespace ippp {

StatsPropertyCollector::StatsPropertyCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsPropertyCollector::setProperties(RobotType robotType, bool useObstacle, bool useConstraint, bool optimized, double stepSize,
                                           SamplerType samplerType, SamplingType samplingType, PlannerType plannerType,
                                           PathModifierType pathModifierType) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_useObstacle = useObstacle;
    m_useConstraint = useConstraint;
    m_optimized = optimized;
    m_stepSize = stepSize;
    m_samplerType = samplerType;
    m_samplingType = samplingType;
    m_plannerType = plannerType;
    m_pathModifierType = pathModifierType;
    m_robotType = robotType;
}

void StatsPropertyCollector::initialize() {
    m_useObstacle = false;
    m_useConstraint = false;
    m_optimized = false;
    m_stepSize = 1;
    m_samplerType = SamplerType::Uniform;
    m_samplingType = SamplingType::Straight;
    m_plannerType = PlannerType::RRT;
    m_pathModifierType = PathModifierType::NodeCut;
    m_robotType = RobotType::Point2D;
}

nlohmann::json StatsPropertyCollector::serialize() {
    nlohmann::json json;
    json["UseObstacle"] = m_useObstacle;
    json["UseConstraint"] = m_useConstraint;
    json["Optimized"] = m_optimized;
    json["StepSize"] = m_stepSize;
    json["SamplerType"] = static_cast<int>(m_samplerType);
    json["SamplingType"] = static_cast<int>(m_samplingType);
    json["PlannerType"] = static_cast<int>(m_plannerType);
    json["PathModifierType"] = static_cast<int>(m_pathModifierType);
    json["RobotType"] = static_cast<int>(m_robotType);
    return json;
}

void StatsPropertyCollector::writeData(std::ostream &stream) {
    stream << getName() << ": " << std::endl;
    stream << "UseObstacle: " << m_useObstacle;
    stream << "UseConstraint: " << m_useConstraint;
    stream << "Optimized: " << m_optimized;
    stream << "StepSize: " << m_stepSize;
    stream << "SamplerType: " << static_cast<int>(m_samplerType);
    stream << "SamplingType: " << static_cast<int>(m_samplingType);
    stream << "PlannerType: " << static_cast<int>(m_plannerType);
    stream << "PathModifierType: " << static_cast<int>(m_pathModifierType);
    stream << "RobotType: " << static_cast<int>(m_robotType);
}

} /* namespace ippp */
