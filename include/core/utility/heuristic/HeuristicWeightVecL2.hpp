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

#ifndef HEURISTICWEIGHTVECL2_H
#define HEURISTICWEIGHTVECL2_H

#include <core/utility/heuristic/Heuristic.hpp>

namespace ippp {

/*!
* \brief   Static class for the computation of heuristic costs from Edge
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class HeuristicWeightVecL2 : public Heuristic<dim> {
  public:
    HeuristicWeightVecL2();
    HeuristicWeightVecL2(const Vector<dim> &weightVec);
    float calcEdgeCost(const Vector<dim> &source, const Vector<dim> &target) const override;

    void setWeightVec(const Vector<dim> &vec);
    Vector<dim> getWeightVec() const;

  private:
    Vector<dim> m_weightVec;
};

/*!
*  \brief      Standard constructor of the class HeuristicWeightVecL2
*  \author     Sascha Kaden
*  \date       2017-02-19
*/
template <unsigned int dim>
HeuristicWeightVecL2<dim>::HeuristicWeightVecL2() : Heuristic<dim>("Heuristic weightVecL2") {
}

/*!
*  \brief      Constructor of the class HeuristicWeightVecL2
*  \author     Sascha Kaden
*  \param[in]  weightVec
*  \date       2017-02-19
*/
template <unsigned int dim>
HeuristicWeightVecL2<dim>::HeuristicWeightVecL2(const Vector<dim> &weightVec) : Heuristic<dim>("Heuristic weightVecL2") {
    setWeightVec(weightVec);
}

/*!
*  \brief      Calculates the heuristic cost of an Edge from the source and target Node by the specified heuristic.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] heuristic cost
*  \date       2017-01-02
*/
template <unsigned int dim>
float HeuristicWeightVecL2<dim>::calcEdgeCost(const Vector<dim> &source, const Vector<dim> &target) const {
    return (source - target).cwiseProduct(m_weightVec).norm();
}

/*!
*  \brief      Sets the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
void HeuristicWeightVecL2<dim>::setWeightVec(const Vector<dim> &vec) {
    m_weightVec = vec;
}

/*!
*  \brief      Returns the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
Vector<dim> HeuristicWeightVecL2<dim>::getWeightVec() const {
    return m_weightVec;
}

} /* namespace ippp */

#endif    // HEURISTICWEIGHTVECL2_H
