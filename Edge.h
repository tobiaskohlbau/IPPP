#ifndef EDGE_H_
#define EDGE_H_

#include <cstdint>
#include <memory>

using std::shared_ptr;

class Node;

class Edge
{
public:
    Edge(const shared_ptr<Node> &parent, const shared_ptr<Node> &target);

    float getLength() const;

    void setSource(const shared_ptr<Node> &parent);
    void setTarget (const shared_ptr<Node> &target);

    shared_ptr<Node> getSource() const;
    shared_ptr<Node> getTarget() const;

private:
    shared_ptr<Node> m_parent;
    shared_ptr<Node> m_target;
};

Edge::Edge(const shared_ptr<Node> &parent, const shared_ptr<Node> &target) {
    m_parent = parent;
    m_target = target;
}

float Edge::getLength() const {
    return 0;//return m_parent->getDist(m_target);
}

void Edge::setSource(const shared_ptr<Node> &parent) {
    m_parent = parent;
}

void Edge::setTarget(const shared_ptr<Node> &target) {
    m_target = target;
}

shared_ptr<Node> Edge::getSource() const {
    return m_parent;
}

shared_ptr<Node> Edge::getTarget() const {
    return m_target;
}

#endif /* EDGE_H_ */
