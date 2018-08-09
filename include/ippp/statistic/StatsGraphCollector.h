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

#ifndef STATSGRAPHCOLLECTOR_H
#define STATSGRAPHCOLLECTOR_H

#include <ippp/statistic/StatsCollector.h>
#include <ippp/statistic/StatsCountContainer.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatsGraphCollector : public StatsCollector {
  public:
    StatsGraphCollector(const std::string &name);

    void setNodeCount(size_t count);
    void setEdgeCount(size_t count);

    virtual void initialize();
    virtual nlohmann::json serialize();
    void writeData(std::ostream &stream);

  private:
    size_t m_nodeCount = 0;
    size_t m_edgeCount = 0;
};

} /* namespace ippp */

#endif    // StatsGraphCollector
