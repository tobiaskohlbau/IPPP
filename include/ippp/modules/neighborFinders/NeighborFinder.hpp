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

#ifndef NEIGHBORFINDER_HPP
#define NEIGHBORFINDER_HPP

#include <ippp/Identifier.h>
#include <ippp/modules/distanceMetrics/DistanceMetric.hpp>
#include <ippp/util/Logging.h>

namespace ippp {

/*!
* \brief   Interface for NeighborFinder
* \author  Sascha Kaden
* \date    2017-05-09
*/
template <unsigned int dim, class T>
class NeighborFinder : public Identifier {
  public:
    NeighborFinder(const std::string &name, const std::shared_ptr<DistanceMetric<dim>> &distanceMetric);
    virtual ~NeighborFinder();

    virtual void addNode(const Vector<dim> &config, const T &node) = 0;
    virtual void rebaseSorted(std::vector<T> &nodes) = 0;

    virtual T searchNearestNeighbor(const Vector<dim> &config) = 0;
    virtual std::vector<T> searchRange(const Vector<dim> &config, double range) = 0;

  protected:
    std::shared_ptr<DistanceMetric<dim>> m_metric;
};

/*!
*  \brief      Default constructor of the class NeighborFinder
*  \author     Sascha Kaden
*  \param[in]  name
*  \date       2017-05-09
*/
template <unsigned int dim, class T>
NeighborFinder<dim, T>::NeighborFinder(const std::string &name, const std::shared_ptr<DistanceMetric<dim>> &distanceMetric)
    : Identifier(name), m_metric(distanceMetric) {
}

template <unsigned int dim, class T>
NeighborFinder<dim, T>::~NeighborFinder() {
    Logging::debug("Initialize", this);
}

} /* namespace ippp */

#endif /* NEIGHBORFINDER_HPP */
