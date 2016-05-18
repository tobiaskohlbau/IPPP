#ifndef EDGE_H_
#define EDGE_H_

#include <cstdint>
#include <memory>

using std::shared_ptr;

template<uint16_t dim>
class Node;

template<uint16_t dim>
class Edge
{
public:
    Edge(const shared_ptr<Node<dim>> &parent, const shared_ptr<Node<dim>> &target);

    float getLength() const;

    void setSource(const shared_ptr<Node<dim>> &parent);
    void setTarget (const shared_ptr<Node<dim>> &target);

    shared_ptr<Node<dim>> getSource() const;
    shared_ptr<Node<dim>> getTarget() const;

private:
    shared_ptr<Node<dim>> m_parent;
    shared_ptr<Node<dim>> m_target;
};

template<uint16_t dim>
Edge<dim>::Edge(const shared_ptr<Node<dim>> &parent, const shared_ptr<Node<dim>> &target) {
    m_parent = parent;
    m_target = target;

}

template<uint16_t dim>
float Edge<dim>::getLength() const {
    return m_parent->getDist(m_target);
}

template<uint16_t dim>
void Edge<dim>::setSource(const shared_ptr<Node<dim>> &parent) {
    m_parent = parent;
}

template<uint16_t dim>
void Edge<dim>::setTarget(const shared_ptr<Node<dim>> &target) {
    m_target = target;
}

template<uint16_t dim>
shared_ptr<Node<dim>> Edge<dim>::getSource() const {
    return m_parent;
}

template<uint16_t dim>
shared_ptr<Node<dim>> Edge<dim>::getTarget() const {
    return m_target;
}

#endif /* EDGE_H_ */
