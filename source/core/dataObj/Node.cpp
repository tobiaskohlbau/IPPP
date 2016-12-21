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

#include <core/dataObj/Node.h>
#include <limits>

using std::shared_ptr;
namespace rmpl {

/*!
*  \brief      Default constructor of the class Node
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
Node::Node() {
    m_cost = 0;
}

/*!
*  \brief      Constructor of the class Node (1D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \date       2016-05-24
*/
Node::Node(float x) {
    m_cost = 0;
    m_vec = Vec<float>(x);
}

/*!
*  \brief      Constructor of the class Node (2D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \date       2016-05-24
*/
Node::Node(float x, float y) {
    m_cost = 0;
    m_vec = Vec<float>(x, y);
}

/*!
*  \brief      Constructor of the class Node (3D)
*  \author     Sascha Kaden
*  \param[in]  x
*  \param[in]  y
*  \param[in]  z
*  \date       2016-05-24
*/
Node::Node(float x, float y, float z) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z);
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
Node::Node(float x, float y, float z, float rx) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx);
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
Node::Node(float x, float y, float z, float rx, float ry) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry);
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
Node::Node(float x, float y, float z, float rx, float ry, float rz) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz);
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
*  \param[in]  wx
*  \date       2016-07-14
*/
Node::Node(float x, float y, float z, float rx, float ry, float rz, float wx) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz, wx);
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
*  \param[in]  wx
*  \param[in]  wy
*  \date       2016-07-14
*/
Node::Node(float x, float y, float z, float rx, float ry, float rz, float wx, float wy) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz, wx, wy);
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
*  \param[in]  wx
*  \param[in]  wy
*  \param[in]  wz
*  \date       2016-07-14
*/
Node::Node(float x, float y, float z, float rx, float ry, float rz, float wx, float wy, float wz) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz, wx, wy, wz);
}

/*!
*  \brief      Constructor of the class Node
*  \author     Sascha Kaden
*  \param[in]  Vector
*  \date       2016-05-24
*/
Node::Node(const Vec<float> &vec) {
    m_vec = vec;
    m_cost = 0;
}

/*!
*  \brief      Return first element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
float Node::getX() {
    assert(m_vec.getDim() > 0);
    return m_vec[0];
}

/*!
*  \brief      Return second element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
float Node::getY() {
    assert(m_vec.getDim() > 1);
    return m_vec[1];
}

/*!
*  \brief      Return third element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
float Node::getZ() {
    assert(m_vec.getDim() > 2);
    return m_vec[2];
}

/*!
*  \brief      Return the dimension of the Node
*  \author     Sascha Kaden
*  \param[out] Dimension of the Node
*  \date       2016-05-24
*/
unsigned int Node::getDim() {
    return m_vec.getDim();
}

/*!
*  \brief      Return true, if the vector is empty
*  \author     Sascha Kaden
*  \param[out] State of the Node
*  \date       2016-05-24
*/
bool Node::empty() const {
    return m_vec.empty();
}

/*!
*  \brief      Set the vector element by index
*  \author     Sascha Kaden
*  \param[in]  value
*  \param[in]  index
*  \date       2016-05-24
*/
void Node::setVecValue(float value, unsigned int index) {
    m_vec[index] = value;
}

/*!
*  \brief      Return the vector element by index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] value
*  \date       2016-05-24
*/
float Node::getVecValue(unsigned int index) {
    return m_vec[index];
}

/*!
*  \brief      Return norm 2 from the vector
*  \author     Sascha Kaden
*  \param[out] norm 2
*  \date       2016-05-24
*/
float Node::norm() {
    return m_vec.norm();
}

/*!
*  \brief      Return distance to passed Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr to Node
*  \param[out] distance
*  \date       2016-05-24
*/
float Node::getDist(const shared_ptr<Node> &node) {
    if (node == nullptr)
        return -1;
    return getDist(*node);
}

/*!
*  \brief      Return distance to passed Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] distance
*  \date       2016-05-24
*/
float Node::getDist(const Node &node) {
    if (node.empty() || empty())
        return -1;
    return m_vec.getDist(node.getVec());
}

/*!
*  \brief      Return distance to parent Node
*  \author     Sascha Kaden
*  \param[out] distance
*  \date       2016-05-24
*/
float Node::getDistToParent() {
    return m_parent->getLength();
}

/*!
*  \brief      Set cost of Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-05-24
*/
void Node::setCost(float cost) {
    if (cost >= 0)
        m_cost = cost;
}

/*!
*  \brief      Add cost to Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-10-22
*/
void Node::addCost(float cost) {
    if (cost >= 0)
        m_cost += cost;
}

/*!
*  \brief      Return cost of Node
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-24
*/
float Node::getCost() {
    return m_cost;
}

/*!
*  \brief      Set parent of Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr parent Node
*  \date       2016-07-15
*/
void Node::setParent(shared_ptr<Node> &parent) {
    if (!parent->empty())
        m_parent = shared_ptr<Edge>(new Edge(std::make_shared<Node>(*this), parent, getDist(parent)));
}

/*!
*  \brief      Return parent Node
*  \author     Sascha Kaden
*  \param[out] shared_ptr parent Node
*  \date       2016-07-15
*/
shared_ptr<Node> Node::getParentNode() {
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
shared_ptr<Edge> Node::getParentEdge() {
    return m_parent;
}

/*!
*  \brief      Set the parent Edge as standard constructor
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
void Node::clearParent() {
    m_parent = nullptr;
}

/*!
*  \brief      Add a child Node to child list
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \date       2016-07-15
*/
void Node::addChild(shared_ptr<Node> &child) {
    if (!child->empty())
        m_childes.push_back(std::shared_ptr<Edge>(new Edge(std::make_shared<Node>(*this), child, getDist(child))));
}

/*!
*  \brief      Return list of child nodes
*  \author     Sascha Kaden
*  \param[out] list of child nodes
*  \date       2016-07-15
*/
std::vector<shared_ptr<Node>> Node::getChildNodes() {
    std::vector<shared_ptr<Node>> childNodes;
    for (auto child : m_childes) {
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
std::vector<std::shared_ptr<Edge>> Node::getChildEdges() {
    return m_childes;
}

/*!
*  \brief      Clear list of child nodes
*  \author     Sascha Kaden
*  \date       2016-07-15
*/
void Node::clearChildes() {
    m_childes.clear();
}

/*!
*  \brief      Return Vec
*  \author     Sascha Kaden
*  \param[out] Vec
*  \date       2016-05-24
*/
Vec<float> Node::getVec() const {
    return m_vec;
}

} /* namespace rmpl */