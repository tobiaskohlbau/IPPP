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

#ifndef STATSCOLLECTOR_H
#define STATSCOLLECTOR_H

#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/statistic/StatsContainer.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatsCollector : public Identifier {
  public:
    StatsCollector(const std::string &name);

    void addContainer(const std::shared_ptr<StatsContainer> &container);
    std::shared_ptr<StatsContainer> getContainer(size_t hash);
    virtual void initialize();

    virtual void writeData(std::ostream &stream) = 0;

  protected:
    std::vector<std::shared_ptr<StatsContainer>> m_containers;
};

} /* namespace ippp */

#endif    // STATSCOLLECTOR_H
