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

#include <ippp/statistic/StatsGraphCollector.h>

namespace ippp {

StatsGraphCollector::StatsGraphCollector(const std::string &name) : StatsCollector(name) {
    initialize();
}

void StatsGraphCollector::setNodeCount(size_t count) {
    m_nodeCount = count;
}

void StatsGraphCollector::setEdgeCount(size_t count) {
    m_edgeCount = count;
}

void StatsGraphCollector::initialize() {
    m_nodeCount = 0;
    m_edgeCount = 0;
}

nlohmann::json StatsGraphCollector::serialize() {
    nlohmann::json json;
    json["NodeCount"] = m_nodeCount;
    json["EdgeCount"] = m_edgeCount;
    return json;
}

void StatsGraphCollector::writeData(std::ostream &stream) {
    stream << getName() << ": " << std::endl;
    stream << "NodeCount: " << m_nodeCount << std::endl;
    stream << "EdgeCount: " << m_edgeCount << std::endl;
}

} /* namespace ippp */
