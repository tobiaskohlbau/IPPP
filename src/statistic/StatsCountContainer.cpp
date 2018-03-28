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

#include <ippp/statistic/StatsCountContainer.h>

namespace ippp {

StatsCountContainer::StatsCountContainer(const std::string &name) : StatsContainer(name) {
    initialize();
}

void StatsCountContainer::initialize() {
    m_count = 0;
    m_counts.clear();
}

void StatsCountContainer::writeData(std::ostream &stream) {
    stream << std::to_string(m_count);
}

void StatsCountContainer::add(size_t num) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_count += num;
}

} /* namespace ippp */
