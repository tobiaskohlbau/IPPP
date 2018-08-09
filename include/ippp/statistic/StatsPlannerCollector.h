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

#ifndef STATSPLANNERCOLLECTOR_H
#define STATSPLANNERCOLLECTOR_H

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
class StatsPlannerCollector : public StatsCollector {
  public:
    StatsPlannerCollector(const std::string &name);

    void startPlannerTimer();
    void stopPlannerTimer();
    void startOptimizationTimer();
    void stopOptimizationTimer();

    virtual void initialize();
    virtual nlohmann::json serialize();
    void writeData(std::ostream &stream);

  private:
    std::chrono::system_clock::time_point m_startPlanning;
    std::chrono::system_clock::time_point m_stopPlanning;
    std::chrono::system_clock::time_point m_startOptimization;
    std::chrono::system_clock::time_point m_stopOptimization;
    std::mutex m_mutex;
};

} /* namespace ippp */

#endif    // STATSPLANNERCOLLECTOR_H
