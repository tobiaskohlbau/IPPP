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

#ifndef NODE_HPP
#define NODE_HPP

#include <assert.h>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include <core/dataObj/Edge.hpp>
#include <core/types.h>
#include <core/utility/UtilVec.hpp>

namespace rmpl {

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
    Node(float x, float y);
    Node(float x, float y, float z);
    Node(float x, float y, float z, float rx);
    Node(float x, float y, float z, float rx, float ry);
    Node(float x, float y, float z, float rx, float ry, float rz);
    Node(const Vector<dim> &vec);

    float getX();
    float getY();
    float getZ();

    bool empty() const;
    void setCost(float cost);
    void addCost(float cost);
    float getCost();

    void setParent(std::shared_ptr<Node> &parent, float edgeCost);
    std::shared_ptr<Node> getParentNode();
    std::shared_ptr<Edge<dim>> getParentEdge();
    void clearParent();
    void addChild(std::shared_ptr<Node> &child, float edgeCost);
    std::vector<std::shared_ptr<Node>> getChildNodes();
    std::vector<std::shared_ptr<Edge<dim>>> getChildEdges();
    void clearChildes();

    Vector<dim> getValues() const;

  private:
    Vector<dim> m_vec;
    float m_cost = -1;

    std::shared_ptr<Edge<dim>> m_parent = nullptr;
    std::vector<std::shared_ptr<Edge<dim>>> m_childes;
};

/*!
*  \brief      Default constructor of the class Node, sets all elements of the Vector NaN.
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node() {
    for (int i = 0; i < dim; ++i)
        m_vec[i] = NAN;
}

/*!
*  \brief      Constructor of the class Node (2D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(float x, float y) {
    m_vec = Eigen::Vector2f(x, y);
}

/*!
*  \brief      Constructor of the class Node (3D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(float x, float y, float z) {
    m_vec = Eigen::Vector3f(x, y, z);
}

/*!
*  \brief      Constructor of the class Node (4D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(float x, float y, float z, float rx) {
    m_vec = Eigen::Vector4f(x, y, z, rx);
}

/*!
*  \brief      Constructor of the class Node (5D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(float x, float y, float z, float rx, float ry) {
    m_vec = utilVec::Vecf(x, y, z, rx, ry);
}

/*!
*  \brief      Constructor of the class Node (6D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \param[in]  rx
*  \param[in]  ry
*  \param[in]  rz
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(float x, float y, float z, float rx, float ry, float rz) {
    m_vec = utilVec::Vecf(x, y, z, rx, ry, rz);
}

/*!
*  \brief      Constructor of the class Node
*  \author     Sascha Kaden
*  \param[in]  Eigen Vector
*  \date       2016-05-24
*/
template <unsigned int dim>
Node<dim>::Node(const Vector<dim> &vec) {
    m_vec = vec;
}

/*!
*  \brief      Return first element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
float Node<dim>::getX() {
    return m_vec[0];
}

/*!
*  \brief      Return second element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
float Node<dim>::getY() {
    return m_vec[1];
}

/*!
*  \brief      Return third element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
template <unsigned int dim>
float Node<dim>::getZ() {
    return m_vec[2];
}

/*!
*  \brief      Return true, if the vector is empty
*  \author     Sascha Kaden
*  \param[out] State of the Node
*  \date       2016-05-24
*/
template <unsigned int dim>
bool Node<dim>::empty() const {
    return std::isnan(m_vec[0]);
}

/*!
*  \brief      Set cost of Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-05-24
*/
template <unsigned int dim>
void Node<dim>::setCost(float cost) {
    if (cost >= 0)
        m_cost = cost;
}

/*!
*  \brief      Add cost to Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-10-22
*/
template <unsigned int dim>
void Node<dim>::addCost(float cost) {
    if (cost >= 0)
        m_cost += cost;
}

/*!
*  \brief      Return cost of the Node
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-24
*/
template <unsigned int dim>
float Node<dim>::getCost() {
    return m_cost;
}

/*!
*  \brief      Set parent of the Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::setParent(std::shared_ptr<Node> &parent, float edgeCost) {
    if (!parent->empty()) {
        std::shared_ptr<Node> node = std::make_shared<Node>(*this);
        m_parent = std::shared_ptr<Edge<dim>>(new Edge<dim>(node, parent, edgeCost));
    }
}

/*!
*  \brief      Return parent Node
*  \author     Sascha Kaden
*  \param[out] shared_ptr of parent Node
*  \date       2016-07-15
*/
template <unsigned int dim>
std::shared_ptr<Node<dim>> Node<dim>::getParentNode() {
    if (!m_parent)
        return nullptr;
    else
        return m_parent->getTarget();
}

/*!
*  \brief      Return parent Edge
*  \author     Sascha Kaden
*  \param[out] Edge
*  \date       2016-10-22
*/
template <unsigned int dim>
std::shared_ptr<Edge<dim>> Node<dim>::getParentEdge() {
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
*  \brief      Add a child Node to the child list
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \date       2016-07-15
*/
template <unsigned int dim>
void Node<dim>::addChild(std::shared_ptr<Node<dim>> &child, float edgeCost) {
    if (!child->empty()) {
        std::shared_ptr<Node> node = std::make_shared<Node>(*this);
        m_childes.push_back(std::shared_ptr<Edge<dim>>(new Edge<dim>(node, child, edgeCost)));
    }
}

/*!
*  \brief      Return list of child nodes
*  \author     Sascha Kaden
*  \param[out] list of child nodes
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Node<dim>>> Node<dim>::getChildNodes() {
    std::vector<std::shared_ptr<Node>> childNodes;
    for (auto child : m_childes)
        childNodes.push_back(child->getTarget());

    return childNodes;
}

/*!
*  \brief      Return list of child edges
*  \author     Sascha Kaden
*  \param[out] list of child edges
*  \date       2016-07-15
*/
template <unsigned int dim>
std::vector<std::shared_ptr<Edge<dim>>> Node<dim>::getChildEdges() {
    return m_childes;
}

/*!
*  \brief      Clear list of childs
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
    return m_vec;
}

} /* namespace rmpl */

#endif /* NODE_HPP */
