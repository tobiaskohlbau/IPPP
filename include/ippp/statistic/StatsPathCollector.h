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

#ifndef STATSPATHCOLLECTOR_H
#define STATSPATHCOLLECTOR_H

#include <chrono>
#include <mutex>
#include <utility>

#include <ippp/statistic/StatsCollector.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatsPathCollector : public StatsCollector {
  public:
    StatsPathCollector(const std::string &name);

    void setNodeCounts(size_t nodeCount, size_t smoothedNodeCount);
    void setConfigCounts(size_t numConfigs, size_t numSmoothedConfigs);
    void setLengths(double length, double smoothedLength);
    void startModificationTimer();
    void stopModificationTimer();

    virtual void initialize();
    virtual nlohmann::json serialize();
    void writeData(std::ostream &stream);

  private:
    size_t m_nodeCount = 0;
    size_t m_smoothedNodeCount = 0;
    size_t m_numConfigs = 0;
    size_t m_numSmoothedConfigs = 0;
    double m_length = 0;
    double m_smoothedLength = 0;
    std::chrono::system_clock::time_point m_startModification;
    std::chrono::system_clock::time_point m_stopModification;
    std::mutex m_mutex;
};

} /* namespace ippp */

#endif    // STATSPATHCOLLECTOR_H
