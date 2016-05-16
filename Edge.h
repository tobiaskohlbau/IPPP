#ifndef EDGE_H_
#define EDGE_H_

#include "Vec.h"
#include <cstdint>
#include <memory>

using std::shared_ptr;

template<uint16_t dim>
class Edge
{
public:
    Edge(const Vec<dim, float> &parent, const Vec<dim, float> &child);

    float getLength() const;

    void setParent(const Vec<dim, float> &parent);
    void setChild (const Vec<dim, float> &child);

    Vec<dim, float> getParent() const;
    Vec<dim, float> getChild() const;

private:
    Vec<dim, float> m_parent;
    Vec<dim, float> m_child;
};

template<uint16_t dim>
Edge<dim>::Edge(const Vec<dim, float> &parent, const Vec<dim, float> &child) {
    m_parent = parent;
    m_child = child;
}

template<uint16_t dim>
float Edge<dim>::getLength() const {
    return m_parent->getDist(m_child);
}

template<uint16_t dim>
void Edge<dim>::setParent(const Vec<dim, float> &parent) {
    m_parent = parent;
}

template<uint16_t dim>
void Edge<dim>::setChild(const Vec<dim, float> &child) {
    m_child = child;
}

template<uint16_t dim>
Vec<dim, float> Edge<dim>::getParent() const {
    return m_parent;
}

template<uint16_t dim>
Vec<dim, float> Edge<dim>::getChild() const {
    return m_child;
}

#endif /* EDGE_H_ */
