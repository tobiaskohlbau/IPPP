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

#ifndef NODE_HPP
#define NODE_HPP

#include <assert.h>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <ippp/core/dataObj/Edge.hpp>
#include <ippp/core/dataObj/NodeDataContainer.h>
#include <ippp/core/types.h>
#include <ippp/core/util/UtilVec.hpp>

namespace ippp {

/*!
* \brief   Class Node to present nodes of the path planner.
* \details Consists of the position by an Vec, a cost parameter, an Edge to the parent and a list of child Edges
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Node {
  public:
    Node();
    Node(const Vector<dim> &config);

    bool empty() const;

    void setCost(double cost);
    void addCost(double cost);
    double getCost() const;

    void setData(const std::shared_ptr<NodeDataContainer> &data);
    std::shared_ptr<NodeDataContainer> getData();
    bool checkData() const;

    void setParent(const std::shared_ptr<Node> &parent, const double edgeCost);
    std::shared_ptr<Node> getParentNode() const;
    std::shared_ptr<Edge<dim>> getParentEdge() const;
    void clearParent();

    void setQueryParent(const std::shared_ptr<Node> &parent, const double edgeCost);
    std::shared_ptr<Node> getQueryParentNode() const;
    std::shared_ptr<Edge<dim>> getQueryParentEdge() const;
    void clearQueryParent();

    void addChild(const std::shared_ptr<Node> &child, const double edgeCost);
    std::vector<std::shared_ptr<Node>> getChildNodes() const;
    std::vector<std::shared_ptr<Edge<dim>>> getChildEdges() const;
    void clearChildes();

    Vector<dim> getValues() const;
    double getValue(const unsigned int index) const;

  private:
    Vector<dim> m_config;
    double m_cost = -1;
    std::shared_ptr<NodeDataContainer> m_data;

    std::shared_ptr<Edge<dim>> m_parent = nullptr;
    std::shared_ptr<Edge<dim>> m_queryParent = nullptr;
    std::vector<std::shared_ptr<Edge<dim>>> m_childes;
};

/*!
*  \brief      Default constructor of the class Node, sets all elements of the Vector NaN.
*  \details    A default Node contains a NAN config and is therefore empty.
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node() {
    for (int i = 0; i < dim; ++i) {
        m_config[i] = NAN;
    }
    m_data = std::shared_ptr<NodeDataContainer>(new NodeDataContainer);
}

/*!
*  \brief      Constructor of the class Node
*  \author     Sascha Kaden
*  \param[in]  Eigen Vector
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(const Vector<dim> &config) {
    m_config = config;
    m_data = std::shared_ptr<NodeDataContainer>(new NodeDataContainer);
}

/*!
*  \brief      Return true, if the vector is empty
*  \author     Sascha Kaden
*  \param[out] State of the Node
*  \date       2016-05-24
*/
template <unsigned int dim>
bool Node<dim>::empty() const {
    return std::isnan(m_config[0]);
}

/*!
*  \brief      Set cost of Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-05-24
*/
template <unsigned int dim>
void Node<dim>::setCost(const double cost) {
    if (cost >= 0) {
        m_cost = cost;
    }
}

/*!
*  \brief      Add cost to Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-10-22
*/
template <unsigned int dim>
void Node<dim>::addCost(const double cost) {
    if (cost >= 0) {
        m_cost += cost;
    }
}

/*!
*  \brief      Return cost of the Node
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-24
*/
template <unsigned int dim>
double Node<dim>::getCost() const {
    return m_cost;
}

/*!
*  \brief      Set data container of the node
*  \author     Sascha Kaden
*  \param[in]  data container
*  \date       2017-06-09
*/
template <unsigned int dim>
void Node<dim>::setData(const std::shared_ptr<NodeDataContainer> &data) {
    m_data = data;
}

/*!
*  \brief      Return data container of the Node
*  \author     Sascha Kaden
*  \param[out] data container
*  \date       2017-06-09
*/
template <unsigned int dim>
std::shared_ptr<NodeDataContainer> Node<dim>::getData() {
    return m_data;
}

/*!
*  \brief      Return true if data container is initialized
*  \author     Sascha Kaden
*  \param[out] validity of the data container
*  \date       2017-06-09
*/
template <unsigned int dim>
bool Node<dim>::checkData() const {
    if (m_data == nullptr)
        return false;
    else
        return true;
}

/*!
*  \brief      Set parent of the Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::setParent(const std::shared_ptr<Node> &parent, const double edgeCost) {
    if (!parent->empty()) {
        std::shared_ptr<Edge<dim>> edge(new Edge<dim>(std::make_shared<Node>(*this), parent, edgeCost));
        m_parent = edge;
    }
}

/*!
*  \brief      Return parent Node
*  \author     Sascha Kaden
*  \param[out] shared_ptr of parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Node<dim>::getParentNode() const {
    if (!m_parent) {
        return nullptr;
    } else {
        return m_parent->getTarget();
    }
}

/*!
*  \brief      Return parent Edge
*  \author     Sascha Kaden
*  \param[out] Edge
*  \date       2016-10-22
*/
template <unsigned int dim>
std::shared_ptr<Edge<dim>> Node<dim>::getParentEdge() const {
    return m_parent;
}

/*!
*  \brief      Removes the parent and set it as nullptr
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearParent() {
    m_parent = nullptr;
}

/*!
*  \brief      Set query parent of the Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr query parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::setQueryParent(const std::shared_ptr<Node> &queryParent, const double edgeCost) {
    if (!queryParent->empty()) {
        std::shared_ptr<Edge<dim>> edge(new Edge<dim>(std::make_shared<Node>(*this), queryParent, edgeCost));
        m_queryParent = edge;
    }
}

/*!
*  \brief      Return query parent Node
*  \author     Sascha Kaden
*  \param[out] shared_ptr of query parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Node<dim>::getQueryParentNode() const {
    if (!m_queryParent) {
        return nullptr;
    } else {
        return m_queryParent->getTarget();
    }
}

/*!
*  \brief      Return query parent Edge
*  \author     Sascha Kaden
*  \param[out] Edge
*  \date       2016-10-22
*/
template <unsigned int dim>
std::shared_ptr<Edge<dim>> Node<dim>::getQueryParentEdge() const {
    return m_queryParent;
}

/*!
*  \brief      Removes the parent and set it as nullptr
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearQueryParent() {
    m_queryParent = nullptr;
}

/*!
*  \brief      Add a child Node to the child list
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::addChild(const std::shared_ptr<Node<dim>> &child, const double edgeCost) {
    if (!child->empty()) {
        std::shared_ptr<Edge<dim>> edge(new Edge<dim>(std::make_shared<Node>(*this), child, edgeCost));
        m_childes.push_back(edge);
    }
}

/*!
*  \brief      Return list of child nodes
*  \author     Sascha Kaden
*  \param[out] list of child nodes
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Node<dim>::getChildNodes() const {
    std::vector<std::shared_ptr<Node>> childNodes;
    for (auto &&child : m_childes) {
        childNodes.push_back(child->getTarget());
    }
    return childNodes;
}

/*!
*  \brief      Return list of child edges
*  \author     Sascha Kaden
*  \param[out] list of child edges
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Edge<dim>>> Node<dim>::getChildEdges() const {
    return m_childes;
}

/*!
*  \brief      Clear list of childes
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearChildes() {
    m_childes.clear();
}

/*!
*  \brief      Return const Vector of the Node
*  \author     Sascha Kaden
*  \param[out] Vec
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> Node<dim>::getValues() const {
    return m_config;
}

/*!
*  \brief      Return value from index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] value
*  \date       2016-05-24
*/
template <unsigned int dim>
double Node<dim>::getValue(const unsigned int index) const {
    return m_config[index];
}

} /* namespace ippp */

#endif /* NODE_HPP */