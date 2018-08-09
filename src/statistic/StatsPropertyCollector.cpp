//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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
#include <ippp/ui/JsonSerializer.hpp>

namespace ippp {

StatsPropertyCollector::StatsPropertyCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsPropertyCollector::setProperties(RobotType robotType, unsigned int dim, bool useObstacle, bool useConstraint,
                                           bool optimized, double stepSize, SamplerType samplerType, SamplingType samplingType,
                                           PlannerType plannerType, PathModifierType pathModifierType, std::pair<Vector6,Vector6> C) {
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
    m_dim = dim;
    m_C = C;
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
    m_dim = 2;
    m_C = std::make_pair(Vector6(),Vector6());
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
    json["Dim"] = m_dim;
    json["C"] = jsonSerializer::serialize(m_C);
    return json;
}

void StatsPropertyCollector::writeData(std::ostream &stream) {
    stream << getName() << ": " << std::endl;
    stream << "UseObstacle: " << m_useObstacle << std::endl;
    stream << "UseConstraint: " << m_useConstraint << std::endl;
    stream << "Optimized: " << m_optimized << std::endl;
    stream << "StepSize: " << m_stepSize << std::endl;
    stream << "SamplerType: " << static_cast<int>(m_samplerType) << std::endl;
    stream << "SamplingType: " << static_cast<int>(m_samplingType) << std::endl;
    stream << "PlannerType: " << static_cast<int>(m_plannerType) << std::endl;
    stream << "PathModifierType: " << static_cast<int>(m_pathModifierType) << std::endl;
    stream << "RobotType: " << static_cast<int>(m_robotType) << std::endl;
    stream << "Dim: " << m_dim << std::endl;
    stream << "Cmin: " << m_C.first << " | Cmax: " << m_C.second << std::endl;
}

} /* namespace ippp */
