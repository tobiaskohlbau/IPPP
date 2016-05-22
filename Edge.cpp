#include "Edge.h"

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
