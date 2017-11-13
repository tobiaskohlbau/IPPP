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

#ifndef STATISTICCOLLECTOR_H
#define STATISTICCOLLECTOR_H

#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include <ippp/Identifier.h>
#include <ippp/statistic/StatisticContainer.h>

namespace ippp {

/*!
* \brief   Statistics class to collect stats and write them
* \author  Sascha Kaden
* \date    2017-10-20
*/
class StatisticCollector : public Identifier {
  public:
    StatisticCollector(const std::string &name);

    void addContainer(const std::shared_ptr<StatisticContainer> &container);
    std::shared_ptr<StatisticContainer> getContainer(const size_t hast);
    void initialize();

    virtual void writeData(std::ostream &stream) = 0;

  private:
    std::vector<std::shared_ptr<StatisticContainer>> m_containers;
};

} /* namespace ippp */

#endif    // STATISTICCOLLECTOR_H
