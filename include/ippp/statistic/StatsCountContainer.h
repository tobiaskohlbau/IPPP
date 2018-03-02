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

#ifndef STATSCOUNTCONTAINER_H
#define STATSCOUNTCONTAINER_H

#include <mutex>
#include <vector>

#include <ippp/statistic/StatsContainer.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatsCountContainer : public StatsContainer {
  public:
    StatsCountContainer(const std::string &name);
    virtual void initialize();
    virtual void writeData(std::ostream &stream);

    void add(size_t num);

  private:
    size_t m_count = 0;
    std::vector<size_t> m_counts;
    std::mutex m_mutex;
};

} /* namespace ippp */

#endif    // STATSCOUNTCONTAINER_H
