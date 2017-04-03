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

#ifndef SRTOPTIONS_HPP
#define SRTOPTIONS_HPP

#include <pathPlanner/options/PlannerOptions.hpp>

namespace rmpl {

/*!
* \brief   Class SRTOptions determines special options for the SRT planner
* \author  Sascha Kaden
* \date    2017-04-02
*/
template <unsigned int dim>
class SRTOptions : public PlannerOptions<dim> {
  public:
    SRTOptions(const unsigned int nbOfTrees, const std::shared_ptr<CollisionDetection<dim>> &collision,
               const std::shared_ptr<TrajectoryPlanner<dim>> &planner, const std::shared_ptr<Sampling<dim>> &sampling,
               const std::shared_ptr<Heuristic<dim>> &heuristic = std::shared_ptr<Heuristic<dim>>(new Heuristic<dim>()),
               const unsigned int sortingCountGraph = 0);

    void setNbOfTrees(const unsigned int nbOfTrees);
    unsigned int getNbOfTrees() const;

  private:
    unsigned int m_nbOfTrees = 10;
};

/*!
*  \brief      Standard constructor of the class PRMOptions
*  \param[in]  rangeSize
*  \param[in]  trajectoryStepSize
*  \param[in]  trajectoryMethod
*  \param[in]  samplerMethod
*  \author     Sascha Kaden
*  \date       2017-04-02
*/
template <unsigned int dim>
SRTOptions<dim>::SRTOptions(const unsigned int nbOfTrees, const std::shared_ptr<CollisionDetection<dim>> &collision,
                            const std::shared_ptr<TrajectoryPlanner<dim>> &planner,
                            const std::shared_ptr<Sampling<dim>> &sampling, const std::shared_ptr<Heuristic<dim>> &heuristic,
                            const unsigned int sortingCountGraph)
    : PlannerOptions<dim>(collision, planner, sampling, heuristic, sortingCountGraph) {
    setNbOfTrees(nbOfTrees);
}

/*!
*  \brief      Sets the range size of the local planner from the PRMPlanner
*  \param[in]  rangeSize
*  \author     Sascha Kaden
*  \date       2017-04-02
*/
template <unsigned int dim>
void SRTOptions<dim>::setNbOfTrees(const unsigned int nbOfTrees) {
    if (nbOfTrees <= 0) {
        Logging::warning("Tree size was smaller than 1 and is set up to 1", this);
        m_nbOfTrees = 1;
    } else {
        m_nbOfTrees = nbOfTrees;
    }
}

/*!
*  \brief      Returns the range size of the local planner from the PRMPlanner
*  \param[out] rangeSize
*  \author     Sascha Kaden
*  \date       2017-04-02
*/
template <unsigned int dim>
unsigned int SRTOptions<dim>::getNbOfTrees() const {
    return m_nbOfTrees;
}

} /* namespace rmpl */

#endif    // SRTOPTIONS_HPP
