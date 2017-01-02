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

namespace rmpl {

template <unsigned int dim>
class Node;

enum EdgeHeuristic { L1, L2, INF };
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
    static float calcNodeCost(const std::shared_ptr<Node<dim>> &node);

    static void setEdgeHeuristic(EdgeHeuristic heuristic);
    static void setNodeHeuristic(NodeHeuristic heuristic);
    static EdgeHeuristic getEdgeHeuristic();
    static NodeHeuristic getNodeHeuristic();

  private:
    static EdgeHeuristic m_edgeHeuristic;
    static NodeHeuristic m_nodeHeuristic;
};

template <unsigned int dim>
EdgeHeuristic Heuristic<dim>::m_edgeHeuristic = EdgeHeuristic::L2;
template <unsigned int dim>
NodeHeuristic Heuristic<dim>::m_nodeHeuristic = NodeHeuristic::norm;

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
    if (m_edgeHeuristic == EdgeHeuristic::L1)
        return (source->getValues() - target->getValues()).sum();
    else if (m_edgeHeuristic == EdgeHeuristic::INF)
        return (source->getValues() - target->getValues()).maxCoeff();
    else
        return (source->getValues() - target->getValues()).norm();
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
    return source->getValues().norm();
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

} /* namespace rmpl */

#endif    // HEURISTIC_H
