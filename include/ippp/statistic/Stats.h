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

#ifndef STATS_H
#define STATS_H

#include <memory>
#include <mutex>
#include <vector>

#include <ippp/statistic/StatsCollector.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class Stats {
  public:
    static void addCollector(const std::shared_ptr<StatsCollector> &collector);
    static std::shared_ptr<StatsCollector> getCollector(size_t hash);
    static std::vector<std::shared_ptr<StatsCollector>> getCollectors();

    static void initializeCollectors();
    static void writeData(std::ostream &stream);

  private:
    static std::vector<std::shared_ptr<StatsCollector>> m_collectors;
};

} /* namespace ippp */

#endif    // STATS_H
