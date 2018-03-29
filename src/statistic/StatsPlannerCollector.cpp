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

#include <ippp/statistic/StatsPlannerCollector.h>

namespace ippp {

StatsPlannerCollector::StatsPlannerCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsPlannerCollector::startPlannerTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_startPlanning = std::chrono::system_clock::now();
}

void StatsPlannerCollector::stopPlannerTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stopPlanning = std::chrono::system_clock::now();
}

void StatsPlannerCollector::startOptimizationTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_startOptimization = std::chrono::system_clock::now();
}

void StatsPlannerCollector::stopOptimizationTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stopOptimization = std::chrono::system_clock::now();
}

void StatsPlannerCollector::initialize() {
    m_startPlanning = std::chrono::system_clock::now();
    m_stopPlanning = std::chrono::system_clock::now();
    m_startOptimization = std::chrono::system_clock::now();
    m_stopOptimization = std::chrono::system_clock::now();
}

nlohmann::json StatsPlannerCollector::serialize() {
    nlohmann::json json;
    std::chrono::duration<double> planning = m_stopPlanning - m_startPlanning;
    std::chrono::duration<double> optimization = m_stopOptimization - m_startOptimization;
    json["PlanningTime"] = planning.count();
    json["OptimizationTime"] = optimization.count();
    return json;
}

void StatsPlannerCollector::writeData(std::ostream &stream) {
    stream << getName() << ": " << std::endl;
    std::chrono::duration<double> planning = m_stopPlanning - m_startPlanning;
    std::chrono::duration<double> optimization = m_stopOptimization - m_startOptimization;
    stream << "PlanningTime: " << planning.count() << std::endl;
    stream << "OptimizationTime: " << optimization.count() << std::endl;
}

} /* namespace ippp */
