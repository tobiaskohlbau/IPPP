#ifndef NODE_H_
#define NODE_H_

#include <assert.h>
#include <cstdint>
#include <memory>
#include "Vec.h"
#include "Edge.h"
#include <vector>

using std::shared_ptr;

/*!
* \brief   Class Node to present nodes of path planner
* \details Consists of the position by an Vec, a cost parameter, pointer to an parent Node and a list of child
* \author  Sascha Kaden
* \date    2016-05-23
*/
class Node
{
public:
    Node();
    Node(const float &x);
    Node(const float &x, const float &y);
    Node(const float &x, const float &y, const float &z);
    Node(const float &x, const float &y, const float &z, const float &rx);
    Node(const float &x, const float &y, const float &z, const float &rx, const float &ry);
    Node(const float &x, const float &y, const float &z, const float &rx, const float &ry, const float &rz);
    Node(const Vec<float> &vec);

    float getX() const;
    float getY() const;
    float getZ() const;

    unsigned int getDim();
    bool empty() const ;
    void setVecValue(const float &value, const unsigned int &index);
    float getVecValue(const unsigned int &index);
    float getDist(const shared_ptr<Node> &node) const;
    float getDist(const Node &node) const;
    float getDistToParent() const;
    float norm() const;

    void setCost(const float &cost);
    float getCost() const;

    void setParent(const shared_ptr<Node> &parent);
    shared_ptr<Node> getParent();
    void clearParent();
    void addChild(const shared_ptr<Node> &child);
    std::vector<shared_ptr<Node>> getChilds();
    void clearChilds();

    void addEdge(Edge edge);
    std::vector<Edge> getEdges();
    void clearEdges();

    Vec<float> getVec() const;


private:
    float        m_cost;
    Vec<float>   m_vec;
    std::vector<Edge>        m_edges;
    shared_ptr<Node>              m_parent;
    std::vector<shared_ptr<Node>> m_childs;
};

/*!
*  \brief      Constructor of the class Node
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
Node::Node(const float &x) {
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
Node::Node(const float &x, const float &y) {
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
Node::Node(const float &x, const float &y, const float &z) {
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
Node::Node(const float &x, const float &y, const float &z, const float &rx) {
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
Node::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry) {
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
Node::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry, const float &rz) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz);
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
float Node::getX() const {
    assert(m_vec.getDim() > 0);
    return m_vec[0];
}

/*!
*  \brief      Return second element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
float Node::getY() const {
    assert(m_vec.getDim() > 1);
    return m_vec[1];
}

/*!
*  \brief      Return third element of the vector
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
float Node::getZ() const {
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
void Node::setVecValue(const float &value, const unsigned int &index) {
    m_vec[index] = value;
}

/*!
*  \brief      Return the vector element by index
*  \author     Sascha Kaden
*  \param[in]  index
*  \param[out] value
*  \date       2016-05-24
*/
float Node::getVecValue(const unsigned int &index) {
    return m_vec[index];
}

/*!
*  \brief      Return norm 2 from the vector
*  \author     Sascha Kaden
*  \param[out] norm 2
*  \date       2016-05-24
*/
float Node::norm() const {
    return m_vec.norm();
}

/*!
*  \brief      Return distance to given Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr to Node
*  \param[out] distance
*  \date       2016-05-24
*/
float Node::getDist(const shared_ptr<Node> &node) const {
    if (node == nullptr)
        return -1;
    return getDist(*node);
}

/*!
*  \brief      Return distance to given Node
*  \author     Sascha Kaden
*  \param[in]  Node
*  \param[out] distance
*  \date       2016-05-24
*/
float Node::getDist(const Node &node) const {
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
float Node::getDistToParent() const {
    if (m_parent == nullptr)
        return -1;
    else
        return getDist(m_parent);
}

/*!
*  \brief      Set cost of Node
*  \author     Sascha Kaden
*  \param[in]  cost
*  \date       2016-05-24
*/
void Node::setCost(const float &cost) {
    if (cost > 0)
        m_cost = cost;
}

/*!
*  \brief      Return cost of Node
*  \author     Sascha Kaden
*  \param[out] cost
*  \date       2016-05-24
*/
float Node::getCost() const {
    return m_cost;
}

/*!
*  \brief      Set parent of Node
*  \author     Sascha Kaden
*  \param[in]  shared_ptr parent Node
*  \date       2016-05-24
*/
void Node::setParent(const shared_ptr<Node> &parent) {
    m_parent = parent;
}

/*!
*  \brief      Return parent of Node
*  \author     Sascha Kaden
*  \param[out] shared_ptr parent Node
*  \date       2016-05-24
*/
shared_ptr<Node> Node::getParent() {
    return m_parent;
}

/*!
*  \brief      Set parent Node to nullptr
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
void Node::clearParent() {
    m_parent = nullptr;
}

/*!
*  \brief      Add a child Node to child list
*  \author     Sascha Kaden
*  \param[in]  shared_ptr child Node
*  \date       2016-05-24
*/
void Node::addChild(const shared_ptr<Node> &child) {
    m_childs.push_back(child);
}

/*!
*  \brief      Return list of child nodes
*  \author     Sascha Kaden
*  \param[out] list of child nodes
*  \date       2016-05-24
*/
std::vector<shared_ptr<Node>> Node::getChilds() {
    return m_childs;
}

/*!
*  \brief      Clear list of child nodes
*  \author     Sascha Kaden
*  \date       2016-05-24
*/
void Node::clearChilds() {
    m_childs.clear();
}

void Node::addEdge(Edge edge) {
    m_edges.push_back(edge);
}

std::vector<Edge> Node::getEdges() {
    return m_edges;
}

void Node::clearEdges() {
    m_edges.clear();
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


#endif /* NODE_H_ */
