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

#include <ippp/statistic/StatsTimeContainer.h>

namespace ippp {

    StatsTimeContainer::StatsTimeContainer(const std::string &name) : StatsContainer(name) {
    initialize();
}

void StatsTimeContainer::initialize() {

}

void StatsTimeContainer::writeData(std::ostream &stream) {
    stream << getDuration().count();
}

void StatsTimeContainer::start() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_start = std::chrono::system_clock::now();
}

void StatsTimeContainer::stop() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stop = std::chrono::system_clock::now();
}

std::chrono::system_clock::time_point StatsTimeContainer::getStart() {
    return m_start;
}

std::chrono::system_clock::time_point StatsTimeContainer::getStop() {
    return m_stop;
}

std::chrono::duration<double> StatsTimeContainer::getDuration() {
    return m_stop - m_start;
}

} /* namespace ippp */
