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

#include <ippp/statistic/Stats.h>

namespace ippp {

std::vector<std::shared_ptr<StatsCollector>> Stats::m_collectors;
std::mutex Stats::m_mutex;

void Stats::addCollector(const std::shared_ptr<StatsCollector> &collector) {
    if (!collector)
        return;
    std::lock_guard<std::mutex> lock(m_mutex);
    m_collectors.push_back(collector);
}

std::shared_ptr<StatsCollector> Stats::getCollector(size_t hash) {
    for (auto &collector : m_collectors)
        if (hash == collector->getHash())
            return collector;

    return nullptr;
}

std::vector<std::shared_ptr<StatsCollector>> Stats::getCollectors() {
    return m_collectors;
}

void Stats::initializeCollectors() {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto &collector : m_collectors)
        collector->initialize();
}

void Stats::clearCollectors() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_collectors.clear();
}

nlohmann::json Stats::serialize() {
    std::lock_guard<std::mutex> lock(m_mutex);
    nlohmann::json json;
    for (auto &collector : m_collectors)
        json[collector->getName()] = collector->serialize();
    return json;
}

void Stats::writeData(std::ostream &stream) {
    std::lock_guard<std::mutex> lock(m_mutex);
    for (auto &collector : m_collectors) {
        collector->writeData(stream);
        stream << std::endl;
    }
}

} /* namespace ippp */
