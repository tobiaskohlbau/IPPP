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

#include <ippp/statistic/StatsTimeCollector.h>
#include <ippp/util/Logging.h>

namespace ippp {

StatsTimeCollector::StatsTimeCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsTimeCollector::start() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_start = std::chrono::system_clock::now();
}

void StatsTimeCollector::stop() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stop = std::chrono::system_clock::now();
}

void StatsTimeCollector::initialize() {
    m_start = std::chrono::system_clock::now();
    m_start = std::chrono::system_clock::now();
}

nlohmann::json StatsTimeCollector::serialize() {
    nlohmann::json json;
    std::chrono::duration<double> duration = m_stop - m_start;
    json[getName()] = duration.count();
    return json;
}

void StatsTimeCollector::writeData(std::ostream &stream) {
    stream << getName() << ": ";
    std::chrono::duration<double> duration = m_stop - m_start;
    stream << duration.count() << std::endl;
}

} /* namespace ippp */
