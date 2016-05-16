#ifndef EDGE_H_
#define EDGE_H_

#include "Node.h"
#include <cstdint>
#include <memory>

using std::shared_ptr;

template<uint16_t dim>
class Edge
{
public:
    Edge(shared_ptr<Node<dim>> parent, shared_ptr<Node<dim>> child);

    float getLength() const;

    void setParent(shared_ptr<Node<dim>> parent);
    void setChild(shared_ptr<Node<dim>> child);

    shared_ptr<Node<dim>> getParent() const;
    shared_ptr<Node<dim>> getChild() const;

private:
    shared_ptr<Node<dim>> m_parent;//parent
    shared_ptr<Node<dim>> m_child;//child
};

template<uint16_t dim>
Edge<dim>::Edge(shared_ptr<Node<dim>> parent, shared_ptr<Node<dim>> child) {
    m_parent = parent;
    m_child = child;
}

template<uint16_t dim>
float Edge<dim>::getLength() const {
    return m_parent->DistanceTo(*m_child);
}

template<uint16_t dim>
void Edge<dim>::setParent(shared_ptr<Node<dim>> parent) {
    m_parent = parent;
}

template<uint16_t dim>
void Edge<dim>::setChild(shared_ptr<Node<dim>> child) {
    m_child = child;
}

template<uint16_t dim>
shared_ptr<Node<dim>> Edge<dim>::getParent() const {
    return m_parent;
}

template<uint16_t dim>
shared_ptr<Node<dim>> Edge<dim>::getChild() const {
    return m_child;
}

#endif /* EDGE_H_ */
