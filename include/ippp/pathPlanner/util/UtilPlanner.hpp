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

#ifndef UTILPLANNER_HPP
#define UTILPLANNER_HPP

#include <ippp/core/dataObj/Graph.hpp>
#include <ippp/core/distanceMetrics/DistanceMetric.hpp>
#include <ippp/core/trajectoryPlanner/TrajectoryPlanner.hpp>

namespace ippp {
namespace util {

/*!
*  \brief      Try to find nearest Node of the graph with a valid connection to the passed config
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[in]  nearest Node
*  \date       2016-08-09
*/
template <unsigned int dim>
static std::shared_ptr<Node<dim>> getNearestValidNode(const Vector<dim> &config, const std::shared_ptr<Graph<dim>> &graph,
                                                      const std::shared_ptr<TrajectoryPlanner<dim>> &trajectory,
                                                      const std::shared_ptr<DistanceMetric<dim>> &metric, const double range) {
    std::shared_ptr<Node<dim>> nearestNode = nullptr;
    std::vector<std::shared_ptr<Node<dim>>> nearNodes = graph->getNearNodes(config, range);
    double dist = std::numeric_limits<double>::max();
    for (auto &nearNode : nearNodes) {
        if (trajectory->checkTrajectory(config, nearNode->getValues()) &&
            metric->calcDist(config, nearNode->getValues()) < dist) {
            dist = metric->calcDist(config, nearNode->getValues());
            nearestNode = nearNode;
        }
    }
    return nearestNode;
}

} /* namespace util */
} /* namespace ippp */

#endif    // UTILPLANNER_HPP
