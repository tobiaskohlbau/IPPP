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

#ifndef STATISTICSIZETCONTAINER_H
#define STATISTICSIZETCONTAINER_H

#include <vector>

#include <ippp/statistic/StatisticContainer.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatisticSizeTContainer : public StatisticContainer {
  public:
    StatisticSizeTContainer(const std::string &name);
    virtual void initialize();

  private:
    size_t m_count = 0;
    std::vector<size_t> m_counts;
};

} /* namespace ippp */

#endif    // STATISTICSIZETCONTAINER_H
