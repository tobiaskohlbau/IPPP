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

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <core/dataObj/Node.hpp>
#include <core/types.h>
#include <core/utility/UtilVec.hpp>

namespace rmpl {

template <unsigned int dim>
class Node;

enum EdgeHeuristic { L1, L2, INF, WeightVec_L1, WeightVec_L2, WeightVec_INF };
enum NodeHeuristic { norm };

/*!
* \brief   Static class for the computation of heuristic costs from Edge and Node
* \author  Sascha Kaden
* \date    2017-01-02
*/
template <unsigned int dim>
class Heuristic {
  public:
    static float calcEdgeCost(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target);
    static float calcEdgeCost(const Vector<dim> &source, const Vector<dim> &target);
    static float calcNodeCost(const std::shared_ptr<Node<dim>> &node);
    static float calcNodeCost(const Vector<dim> &node);

    static void setEdgeHeuristic(EdgeHeuristic heuristic);
    static void setNodeHeuristic(NodeHeuristic heuristic);
    static EdgeHeuristic getEdgeHeuristic();
    static NodeHeuristic getNodeHeuristic();

    static void setWeightVec(const Vector<dim> &vec);
    static Vector<dim> getWeightVec();

  private:
    static EdgeHeuristic m_edgeHeuristic;
    static NodeHeuristic m_nodeHeuristic;
    static Vector<dim> m_weightVec;
};

template <unsigned int dim>
EdgeHeuristic Heuristic<dim>::m_edgeHeuristic = EdgeHeuristic::L2;
template <unsigned int dim>
NodeHeuristic Heuristic<dim>::m_nodeHeuristic = NodeHeuristic::norm;
template <unsigned int dim>
Vector<dim> Heuristic<dim>::m_weightVec = utilVec::Vecf<dim>(1);

/*!
*  \brief      Calculates the heuristic cost of an Edge from the source and target Node by the specified heuristic.
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[out] heuristic cost
*  \date       2017-01-02
*/
template <unsigned int dim>
float Heuristic<dim>::calcEdgeCost(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target) {
    return calcEdgeCost(source->getValues(), target->getValues());
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
float Heuristic<dim>::calcEdgeCost(const Vector<dim> &source, const Vector<dim> &target) {
    if (m_edgeHeuristic == EdgeHeuristic::L1) {
        return (source - target).sum();
    } else if (m_edgeHeuristic == EdgeHeuristic::INF) {
        return (source - target).maxCoeff();
    } else if (m_edgeHeuristic == EdgeHeuristic::WeightVec_L1) {
        return (source - target).cwiseProduct(m_weightVec).sum();
    } else if (m_edgeHeuristic == EdgeHeuristic::WeightVec_L2) {
        return (source - target).cwiseProduct(m_weightVec).norm();
    } else if (m_edgeHeuristic == EdgeHeuristic::WeightVec_INF) {
        return (source - target).cwiseProduct(m_weightVec).maxCoeff();
    } else {
        return (source - target).norm();
    }
}

/*!
*  \brief      Calculates the heuristic cost of a Node by the specified heuristic.
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] heuristic cost
*  \date       2017-01-02
*/
template <unsigned int dim>
float Heuristic<dim>::calcNodeCost(const std::shared_ptr<Node<dim>> &source) {
    return calcNodeCost(source->getValues());
}

/*!
*  \brief      Calculates the heuristic cost of a Node by the specified heuristic.
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] heuristic cost
*  \date       2017-01-02
*/
    template <unsigned int dim>
    float Heuristic<dim>::calcNodeCost(const Vector<dim> &source) {
        return source.norm();
    }

/*!
*  \brief      Sets the heuristic method for the Edge
*  \author     Sascha Kaden
*  \param[in]  method
*  \date       2017-01-02
*/
template <unsigned int dim>
void Heuristic<dim>::setEdgeHeuristic(EdgeHeuristic heuristic) {
    m_edgeHeuristic = heuristic;
}

/*!
*  \brief      Sets the heuristic method for the Node
*  \author     Sascha Kaden
*  \param[in]  method
*  \date       2017-01-02
*/
template <unsigned int dim>
void Heuristic<dim>::setNodeHeuristic(NodeHeuristic heuristic) {
    m_nodeHeuristic = heuristic;
}

/*!
*  \brief      Returns the heuristic method from the Edge
*  \author     Sascha Kaden
*  \param[in]  method
*  \date       2017-01-02
*/
template <unsigned int dim>
EdgeHeuristic Heuristic<dim>::getEdgeHeuristic() {
    return m_edgeHeuristic;
}

/*!
*  \brief      Returns the heuristic method from the Node
*  \author     Sascha Kaden
*  \param[in]  method
*  \date       2017-01-02
*/
template <unsigned int dim>
NodeHeuristic Heuristic<dim>::getNodeHeuristic() {
    return m_nodeHeuristic;
}

/*!
*  \brief      Sets the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
void Heuristic<dim>::setWeightVec(const Vector<dim> &vec) {
    m_weightVec = vec;
}

/*!
*  \brief      Returns the weighting Vector
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2017-01-02
*/
template <unsigned int dim>
Vector<dim> Heuristic<dim>::getWeightVec() {
    return m_weightVec;
}

} /* namespace rmpl */

#endif    // HEURISTIC_H
