//-------------------------------------------------------------------------//
//
// Copyright 2016 Sascha Kaden
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

#include <core/utility/Logging.h>
#include <pathPlanner/options/PlannerOptions.h>

namespace rmpl {

/*!
*  \brief      Standard constructor of the class PlannerOptions
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \param[in]  samplingMethod
*  \param[in]  edgeHeuristic
*  \param[in]  nodeHeuristic
*  \param[in]  sortingCountGraph
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
PlannerOptions::PlannerOptions(float trajectoryStepSize, SamplerMethod method, SamplingStrategy strategy,
                               EdgeHeuristic edgeHeuristic, NodeHeuristic nodeHeuristic, unsigned int sortingCountGraph) {
    setTrajectoryStepSize(trajectoryStepSize);
    m_samplingStrategy = strategy;
    m_samplerMethod = method;
    m_edgeHeuristic = edgeHeuristic;
    m_nodeHeuristic = nodeHeuristic;
    m_sortingCountGraph = sortingCountGraph;
}

/*!
*  \brief      Sets the trajectory step size
*  \param[in]  stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void PlannerOptions::setTrajectoryStepSize(float stepSize) {
    if (stepSize <= 0) {
        Logging::warning("Trajectory step size was smaller than 0 and was set up to 1", "PlannerOptions");
        m_trajectoryStepSize = 1;
    } else {
        m_trajectoryStepSize = stepSize;
    }
}

/*!
*  \brief      Returns the trajectory step size
*  \param[out] stepSize
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
float PlannerOptions::getTrajectoryStepSize() const {
    return m_trajectoryStepSize;
}

/*!
*  \brief      Sets the sampling method
*  \param[in]  method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
void PlannerOptions::setSamplerMethod(SamplerMethod method) {
    m_samplerMethod = method;
}

/*!
*  \brief      Returns the sampling method
*  \param[out] method
*  \author     Sascha Kaden
*  \date       2016-08-29
*/
SamplerMethod PlannerOptions::getSamplerMethod() const {
    return m_samplerMethod;
}

/*!
*  \brief      Sets the sampling strategy
*  \param[in]  strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
void PlannerOptions::setSamplingStrategy(SamplingStrategy strategy) {
    m_samplingStrategy = strategy;
}

/*!
*  \brief      Returns the strategy method
*  \param[out] strategy
*  \author     Sascha Kaden
*  \date       2016-12-15
*/
SamplingStrategy PlannerOptions::getSamplingStrategy() const {
    return m_samplingStrategy;
}

/*!
*  \brief      Sets the EdgeHeuristic
*  \param[in]  heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
void PlannerOptions::setEdgeHeuristic(EdgeHeuristic heuristic) {
    m_edgeHeuristic = heuristic;
}

/*!
*  \brief      Returns the EdgeHeuristic
*  \param[out] heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
EdgeHeuristic PlannerOptions::getEdgeHeuristic() const {
    return m_edgeHeuristic;
}

/*!
*  \brief      Sets the NodeHeuristic
*  \param[in]  heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
void PlannerOptions::setNodeHeuristic(NodeHeuristic heuristic) {
    m_nodeHeuristic = heuristic;
}

/*!
*  \brief      Returns the NodeHeuristic
*  \param[out] heuristic
*  \author     Sascha Kaden
*  \date       2017-01-01
*/
NodeHeuristic PlannerOptions::getNodeHeuristic() const {
    return m_nodeHeuristic;
}

/*!
*  \brief      Sets count to sort Graph automically
*  \param[in]  autoSort
*  \author     Sascha Kaden
*  \date       2017-01-05
*/
void PlannerOptions::setSortCountGraph(unsigned int sortingCountGraph) {
    m_sortingCountGraph = sortingCountGraph;
}

/*!
*  \brief      Returns count to sort Graph automically
*  \param[out] autoSort
*  \author     Sascha Kaden
*  \date       2017-01-05
*/
unsigned int PlannerOptions::getSortCountGraph() const {
    return m_sortingCountGraph;
}

} /* namespace rmpl */