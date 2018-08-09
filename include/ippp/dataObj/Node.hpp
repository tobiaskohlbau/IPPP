//-------------------------------------------------------------------------//
//
// Copyright 2018 Sascha Kaden
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

#include <ippp/types.h>
#include <ippp/util/UtilVec.hpp>

namespace ippp {

/*!
* \brief   Class Node to present nodes of the MotionPlanner inside of the Graph.
* \details The Node contains a configuration by a Vector, furthermore does it contain all edges of the Graph the and a optional
* cost parameter.
* \author  Sascha Kaden
* \date    2016-05-23
*/
template <unsigned int dim>
class Node {
  public:
    Node();
    Node(const Vector<dim> &config);

    bool empty() const;
    void clearPointer();

    void setPathCost(double pathCost);
    void addPathCost(double pathCost);
    double getPathCost() const;

    void setStateCost(double stateCost);
    void addStateCost(double stateCost);
    double getStateCost() const;

    void setParent(const std::shared_ptr<Node> &parent, double edgeCost);
    std::shared_ptr<Node> getParentNode() const;
    std::pair<std::shared_ptr<Node<dim>>, double> getParentEdge() const;
    void clearParent();

    void setQueryParent(const std::shared_ptr<Node> &parent, double edgeCost);
    std::shared_ptr<Node> getQueryParentNode() const;
    std::pair<std::shared_ptr<Node<dim>>, double> getQueryParentEdge() const;
    void clearQueryParent();

    void addChild(const std::shared_ptr<Node> &child, double edgeCost);
    std::vector<std::shared_ptr<Node>> getChildNodes() const;
    std::vector<std::pair<std::shared_ptr<Node<dim>>, double>> getChildEdges() const;
    size_t getChildSize() const;
    bool isChild(const std::shared_ptr<Node> &child) const;
    void clearChildren();

    void addInvalidChild(const std::shared_ptr<Node> &child);
    bool isInvalidChild(const std::shared_ptr<Node> &node) const;
    void clearInvalidChildren();

    Vector<dim> getValues() const;
    double getValue(unsigned int index) const;

  private:
    Vector<dim> m_config; /*!< configuration of the Node */
    double m_pathCost = 0;    /*!< path cost parameter */
    double m_stateCost = 0;    /*!< state cost parameter */

    std::pair<std::shared_ptr<Node<dim>>, double> m_parent =
        std::make_pair(nullptr, 0); /*!< parent edge (pointer to the node + edge cost) */
    std::pair<std::shared_ptr<Node<dim>>, double> m_queryParent =
        std::make_pair(nullptr, 0); /*!< query parent edge (pointer to the node + edge cost) */
    std::vector<std::pair<std::shared_ptr<Node<dim>>, double>> m_children; /*!< child edges (pointer to the node + edge cost) */
    std::vector<std::shared_ptr<Node>> m_invalidChildren; /*!< list of near children, with them no connection was possible */
};

/*!
*  \brief      Default constructor of the class Node, sets all elements of the Vector NaN.
*  \details    A default Node contains a NAN config and is therefore empty.
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node() {
    for (unsigned int i = 0; i < dim; ++i)
        m_config[i] = NAN;
}

/*!
*  \brief      Constructor of the class Node.
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(const Vector<dim> &config) {
    m_config = config;
}

/*!
*  \brief      Return true, if the vector is empty.
*  \author     Sascha Kaden
*  \param[out] State of the Node
*  \date       2016-05-24
*/
template <unsigned int dim>
bool Node<dim>::empty() const {
    return std::isnan(m_config[0]);
}

/*!
*  \brief      Clear all pointers to other nodes.
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
void Node<dim>::clearPointer() {
    clearParent();
    clearQueryParent();
    clearChildren();
    clearInvalidChildren();
}

/*!
*  \brief      Set path cost of Node.
*  \author     Sascha Kaden
*  \param[in]  pathCost
*  \date       2016-05-24
*/
template <unsigned int dim>
void Node<dim>::setPathCost(double pathCost) {
    if (pathCost >= 0)
        m_pathCost = pathCost;
}

/*!
*  \brief      Add path path cost to Node.
*  \author     Sascha Kaden
*  \param[in]  pathCost
*  \date       2016-10-22
*/
template <unsigned int dim>
void Node<dim>::addPathCost(double pathCost) {
    if (pathCost > 0)
        m_pathCost += pathCost;
}

/*!
*  \brief      Return path cost of the Node.
*  \author     Sascha Kaden
*  \param[out] pathCost
*  \date       2016-05-24
*/
template <unsigned int dim>
double Node<dim>::getPathCost() const {
    return m_pathCost;
}

/*!
*  \brief      Set state cost of Node.
*  \author     Sascha Kaden
*  \param[in]  stateCost
*  \date       2018-08-09
*/
template <unsigned int dim>
void Node<dim>::setStateCost(double stateCost) {
    if (stateCost >= 0)
        m_stateCost = stateCost;
}

/*!
*  \brief      Add path state cost to Node.
*  \author     Sascha Kaden
*  \param[in]  stateCost
*  \date       2018-08-09
*/
template <unsigned int dim>
void Node<dim>::addStateCost(double stateCost) {
    if (stateCost > 0)
        m_stateCost += stateCost;
}

/*!
*  \brief      Return state cost of the Node.
*  \author     Sascha Kaden
*  \param[out] stateCost
*  \date       2018-08-09
*/
template <unsigned int dim>
double Node<dim>::getStateCost() const {
    return m_stateCost;
}

/*!
*  \brief      Set parent of the Node.
*  \author     Sascha Kaden
*  \param[in]  parent Node
*  \param[in]  edgeCost
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::setParent(const std::shared_ptr<Node> &parent, double edgeCost) {
    if (!parent->empty())
        m_parent = std::make_pair(parent, edgeCost);
}

/*!
*  \brief      Return parent Node.
*  \author     Sascha Kaden
*  \param[out] parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Node<dim>::getParentNode() const {
    return m_parent.first;
}

/*!
*  \brief      Return parent Edge.
*  \author     Sascha Kaden
*  \param[out] Edge
*  \date       2016-10-22
*/
template <unsigned int dim>
std::pair<std::shared_ptr<Node<dim>>, double> Node<dim>::getParentEdge() const {
    return m_parent;
}

/*!
*  \brief      Removes the parent and set it to nullptr.
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearParent() {
    m_parent = std::make_pair(nullptr, 0);
}

/*!
*  \brief      Set query parent of the Node.
*  \author     Sascha Kaden
*  \param[in]  shared_ptr query parent Node
*  \param[in]  edgeCost
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::setQueryParent(const std::shared_ptr<Node> &queryParent, double edgeCost) {
    if (!queryParent->empty())
        m_queryParent = std::make_pair(queryParent, edgeCost);
}

/*!
*  \brief      Return query parent Node.
*  \author     Sascha Kaden
*  \param[out] shared_ptr of query parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Node<dim>::getQueryParentNode() const {
    return m_queryParent.first;
}

/*!
*  \brief      Return query parent Edge.
*  \author     Sascha Kaden
*  \param[out] Edge
*  \date       2016-10-22
*/
template <unsigned int dim>
std::pair<std::shared_ptr<Node<dim>>, double> Node<dim>::getQueryParentEdge() const {
    return m_queryParent;
}

/*!
*  \brief      Removes the parent and set it to nullptr.
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearQueryParent() {
    m_queryParent = std::make_pair(nullptr, -1);
}

/*!
*  \brief      Add a child Node to the child list.
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \param[in]  edgeCost
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::addChild(const std::shared_ptr<Node<dim>> &child, double edgeCost) {
    m_children.push_back(std::make_pair(child, edgeCost));
}

/*!
*  \brief      Return list of child nodes.
*  \author     Sascha Kaden
*  \param[out] list of child nodes
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Node<dim>::getChildNodes() const {
    std::vector<std::shared_ptr<Node>> childNodes;
    for (auto &&child : m_children)
        childNodes.push_back(child.first);

    return childNodes;
}

/*!
*  \brief      Return list of child edges.
*  \author     Sascha Kaden
*  \param[out] list of child edges
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::pair<std::shared_ptr<Node<dim>>, double>> Node<dim>::getChildEdges() const {
    return m_children;
}

/*!
*  \brief      Return size of the child edges.
*  \author     Sascha Kaden
*  \param[out] child size
*  \date       2017-10-24
*/
template <unsigned int dim>
size_t Node<dim>::getChildSize() const {
    return m_children.size();
}

/*!
*  \brief      Check if passed node is child.
*  \author     Sascha Kaden
*  \param[in]  node
*  \param[out] true if existing child
*  \date       2017-10-12
*/
template <unsigned int dim>
bool Node<dim>::isChild(const std::shared_ptr<Node<dim>> &node) const {
    for (auto &child : m_children)
        if (child.first == node)
            return true;

    return false;
}

/*!
*  \brief      Clear list of children.
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::clearChildren() {
    m_children.clear();
}

/*!
*  \brief      Add an invalid child Node to the invalid children list.
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \date       2017-10-12
*/
template <unsigned int dim>
void Node<dim>::addInvalidChild(const std::shared_ptr<Node<dim>> &child) {
    if (child && child->empty())
        return;

    m_invalidChildren.push_back(child);
}

/*!
*  \brief      Check if passed node is invalid child of the Node.
*  \author     Sascha Kaden
*  \param[in]  node
*  \param[out] true if existing invalid child
*  \date       2017-10-12
*/
template <unsigned int dim>
bool Node<dim>::isInvalidChild(const std::shared_ptr<Node<dim>> &node) const {
    for (auto &child : m_invalidChildren)
        if (child == node)
            return true;

    return false;
}

/*!
*  \brief      Clear list of invalid children.
*  \author     Sascha Kaden
*  \date       2017-10-12
*/
template <unsigned int dim>
void Node<dim>::clearInvalidChildren() {
    m_invalidChildren.clear();
}

/*!
*  \brief      Return const Vector of the Node.
*  \author     Sascha Kaden
*  \param[out] Vec
*  \date       2016-05-24
*/
template <unsigned int dim>
Vector<dim> Node<dim>::getValues() const {
    return m_config;
}

/*!
*  \brief      Return value from index.
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] value
*  \date       2016-05-24
*/
template <unsigned int dim>
double Node<dim>::getValue(unsigned int index) const {
    return m_config[index];
}

} /* namespace ippp */

#endif /* NODE_HPP */
