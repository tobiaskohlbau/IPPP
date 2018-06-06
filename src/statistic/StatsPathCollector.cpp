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

#include <ippp/statistic/StatsPathCollector.h>

namespace ippp {

StatsPathCollector::StatsPathCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsPathCollector::setNodeCounts(size_t nodeCount, size_t smoothedNodeCount) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_nodeCount = nodeCount;
    m_smoothedNodeCount = smoothedNodeCount;
}

void StatsPathCollector::setConfigCounts(size_t numConfigs, size_t numSmoothedConfigs) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_numConfigs = numConfigs;
    m_numSmoothedConfigs = numSmoothedConfigs;
}

void StatsPathCollector::setLengths(double length, double smoothedLength) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_length = length;
    m_smoothedLength = smoothedLength;
}

void StatsPathCollector::startModificationTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_startModification = std::chrono::system_clock::now();
}

void StatsPathCollector::stopModificationTimer() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stopModification = std::chrono::system_clock::now();
}

void StatsPathCollector::initialize() {
    m_nodeCount = 0;
    m_smoothedNodeCount = 0;
    m_numConfigs = 0;
    m_numSmoothedConfigs = 0;
    m_length = 0;
    m_smoothedLength = 0;
    m_startModification = std::chrono::system_clock::now();
    m_stopModification = std::chrono::system_clock::now();
}

nlohmann::json StatsPathCollector::serialize() {
    nlohmann::json json;
    json["NodeCount"] = m_nodeCount;
    json["SmoothedNodeCount"] = m_smoothedNodeCount;
    json["NumConfigs"] = m_numConfigs;
    json["NumSmoothedConfigs"] = m_numSmoothedConfigs;
    json["Length"] = m_length;
    json["SmoothedLength"] = m_smoothedLength;
    std::chrono::duration<double> duration = m_stopModification - m_startModification;
    json["ModificationTime"] = duration.count();
    return json;
}

void StatsPathCollector::writeData(std::ostream &stream) {
    stream << getName() << ": " << std::endl;
    stream << "NodeCount: " << m_nodeCount << std::endl;
    stream << "SmoothedNodeCount: " << m_smoothedNodeCount << std::endl;
    stream << "NumConfigs: " << m_numConfigs << std::endl;
    stream << "NumSmoothedConfigs: " << m_numSmoothedConfigs << std::endl;
    stream << "Length: " << m_length << std::endl;
    stream << "SmoothedLength: " << m_smoothedLength << std::endl;
    std::chrono::duration<double> duration = m_stopModification - m_startModification;
    stream << "ModificationTime: " << duration.count() << std::endl;
}

} /* namespace ippp */
