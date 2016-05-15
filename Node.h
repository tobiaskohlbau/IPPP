#ifndef NODE_H_
#define NODE_H_

#include <assert.h>
#include <cstdint>
#include <memory>
#include "Vec.h"

template <uint16_t dim>
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
    Node(const Vec<dim, float> &vec);

    float getX() const;
    float getY() const;
    float getZ() const;

    bool empty() const ;
    void setVecValue(const float &value, const uint16_t &index);
    float getDist(const std::shared_ptr<Node<dim>> &node) const;
    float getDist(const Node<dim> &node) const;
    float getDistToParent() const;
    float norm() const;

    void setCost(const float &cost);
    float getCost() const;

    void setParent(const std::shared_ptr<Node<dim>> &parent);
    std::shared_ptr<Node<dim>> getParent();
    void addChild(const std::shared_ptr<Node<dim>> &child);
    std::vector<std::shared_ptr<Node<dim>>> getChilds();

    Vec<dim, float> getVec() const;

private:
    float m_cost;
    std::shared_ptr<Node<dim>> m_parent;
    std::vector<std::shared_ptr<Node<dim>>> m_childs;
    Vec<dim, float> m_vec;
};

template<uint16_t dim>
Node<dim>::Node() {
    m_cost = 0;
}

template<uint16_t dim>
Node<dim>::Node(const float &x) {
    assert(dim == 1);
    m_cost = 0;
    m_vec = Vec<dim, float>(x);
}

template<uint16_t dim>
Node<dim>::Node(const float &x, const float &y) {
    assert(dim == 2);
    m_cost = 0;
    m_vec = Vec<dim, float>(x, y);
}

template<uint16_t dim>
Node<dim>::Node(const float &x, const float &y, const float &z) {
    assert(dim == 3);
    m_cost = 0;
    m_vec = Vec<dim, float>(x, y, z);
}

template<uint16_t dim>
Node<dim>::Node(const float &x, const float &y, const float &z, const float &rx) {
    assert(dim == 4);
    m_cost = 0;
    m_vec = Vec<dim, float>(x, y, z, rx);
}

template<uint16_t dim>
Node<dim>::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry) {
    assert(dim == 5);
    m_cost = 0;
    m_vec = Vec<dim, float>(x, y, z, rx, ry);
}

template<uint16_t dim>
Node<dim>::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry, const float &rz) {
    assert(dim == 6);
    m_cost = 0;
    m_vec = Vec<dim, float>(x, y, z, rx, ry, rz);
}

template<uint16_t dim>
Node<dim>::Node(const Vec<dim, float> &vec) {
    m_vec = Vec<dim, float>(vec);
    m_cost = 0;
}

template<uint16_t dim>
float Node<dim>::getX() const {
    assert(dim > 0);
    return m_vec[0];
}

template<uint16_t dim>
float Node<dim>::getY() const {
    assert(dim > 1);
    return m_vec[1];
}

template<uint16_t dim>
float Node<dim>::getZ() const {
    assert(dim > 2);
    return m_vec[2];
}

template<uint16_t dim>
bool Node<dim>::empty() const {
    return m_vec.empty();
}

template<uint16_t dim>
void Node<dim>::setVecValue(const float &value, const uint16_t &index) {
    m_vec[index] = value;
}

template<uint16_t dim>
float Node<dim>::norm() const {
    return m_vec.norm();
}

template<uint16_t dim>
float Node<dim>::getDist(const std::shared_ptr<Node<dim>> &node) const {
    return getDist(*node);
}

template<uint16_t dim>
float Node<dim>::getDist(const Node<dim> &node) const {
    return m_vec.getDist(node.getVec());
}

template<uint16_t dim>
float Node<dim>::getDistToParent() const {
    if (m_parent == NULL)
        return -1;
    else
        return getDist(m_parent);
}

template<uint16_t dim>
void Node<dim>::setCost(const float &cost) {
    if (cost > 0)
        m_cost = cost;
}

template<uint16_t dim>
float Node<dim>::getCost() const {
    return m_cost;
}

template<uint16_t dim>
void Node<dim>::setParent(const std::shared_ptr<Node<dim>> &parent) {
    m_parent = parent;
}

template<uint16_t dim>
std::shared_ptr<Node<dim>> Node<dim>::getParent() {
    return m_parent;
}

template<uint16_t dim>
void Node<dim>::addChild(const std::shared_ptr<Node<dim>> &child) {
    m_childs.push_back(child);
}

template<uint16_t dim>
std::vector<std::shared_ptr<Node<dim>>> Node<dim>::getChilds() {
    return m_childs;
}

template<uint16_t dim>
Vec<dim, float> Node<dim>::getVec() const {
    return m_vec;
}


#endif /* NODE_H_ */
