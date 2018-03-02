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

#include <ippp/statistic/StatsCollisionCollector.h>

namespace ippp {

StatsCollisionCollector::StatsCollisionCollector(const std::string &name) : StatsCollector(name) {
    m_count = std::make_shared<StatsCountContainer>("CollisionCount");
    m_containers.push_back(m_count);
}

void StatsCollisionCollector::writeData(std::ostream &stream) {
    stream << getName() << ": ";
    m_count->writeData(stream);
    stream << std::endl;
}

void StatsCollisionCollector::add(size_t num) {
    m_count->add(num);
}

} /* namespace ippp */
