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

#ifndef EDGE_HPP
#define EDGE_HPP

#include <memory>

#include <ippp/dataObj/Node.hpp>

namespace ippp {

template <unsigned int dim>
class Node;

/*!
* \brief   Class Edge consists of the source and the target Node. It contains and calculates the cost between the Nodes.
* \author  Sascha Kaden
* \date    2016-05-25
*/
template <unsigned int dim>
class Edge {
  public:
    Edge(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target, double cost);

    void setCost(double cost);
    double getCost() const;

    void setSource(const std::shared_ptr<Node<dim>> &source);
    std::shared_ptr<Node<dim>> getSource() const;
    void setTarget(const std::shared_ptr<Node<dim>> &target);
    std::shared_ptr<Node<dim>> getTarget() const;

  private:
    std::shared_ptr<Node<dim>> m_source = nullptr;
    std::shared_ptr<Node<dim>> m_target = nullptr;
    double m_cost = -1;
};

/*!
*  \brief      Constructor of the class Edge
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \param[in]  target Node
*  \param[in]  cost
*  \date       2016-05-25
*/
template <unsigned int dim>
Edge<dim>::Edge(const std::shared_ptr<Node<dim>> &source, const std::shared_ptr<Node<dim>> &target, const double cost) {
    m_source = source;
    m_target = target;
    m_cost = cost;
}
/*!
*  \brief      Return the euclidean length of the Edge
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-25
*/
template <unsigned int dim>
void Edge<dim>::setCost(const double cost) {
    m_cost = cost;
}

/*!
*  \brief      Return the euclidean length of the Edge
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-25
*/
template <unsigned int dim>
double Edge<dim>::getCost() const {
    return m_cost;
}

/*!
*  \brief      Set target Node of the Edge
*  \author     Sascha Kaden
*  \param[in]  target Node
*  \date       2016-05-25
*/
template <unsigned int dim>
void Edge<dim>::setTarget(const std::shared_ptr<Node<dim>> &target) {
    m_target = target;
}

/*!
*  \brief      Return target Node of the Edge
*  \author     Sascha Kaden
*  \param[out] target Node
*  \date       2016-05-25
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Edge<dim>::getTarget() const {
    return m_target;
}

/*!
*  \brief      Set source Node of the Edge
*  \author     Sascha Kaden
*  \param[in]  source Node
*  \date       2016-12-16
*/
template <unsigned int dim>
void Edge<dim>::setSource(const std::shared_ptr<Node<dim>> &source) {
    m_source = source;
}

/*!
*  \brief      Return source Node of the Edge
*  \author     Sascha Kaden
*  \param[out] source Node
*  \date       2016-12-16
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Edge<dim>::getSource() const {
    return m_source;
}

} /* namespace ippp */

#endif /* EDGE_HPP */
