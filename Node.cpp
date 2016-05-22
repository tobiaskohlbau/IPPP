#include "Node.h"

Node::Node() {
    m_cost = 0;
}

Node::Node(const float &x) {
    m_cost = 0;
    m_vec = Vec<float>(x);
}

Node::Node(const float &x, const float &y) {
    m_cost = 0;
    m_vec = Vec<float>(x, y);
}

Node::Node(const float &x, const float &y, const float &z) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z);
}

Node::Node(const float &x, const float &y, const float &z, const float &rx) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx);
}

Node::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry);
}

Node::Node(const float &x, const float &y, const float &z, const float &rx, const float &ry, const float &rz) {
    m_cost = 0;
    m_vec = Vec<float>(x, y, z, rx, ry, rz);
}

Node::Node(const Vec<float> &vec) {
    m_vec = vec;
    m_cost = 0;
}

float Node::getX() const {
    assert(m_vec.getDim() > 0);
    return m_vec[0];
}

float Node::getY() const {
    assert(m_vec.getDim() > 1);
    return m_vec[1];
}

float Node::getZ() const {
    assert(m_vec.getDim() > 2);
    return m_vec[2];
}

unsigned int Node::getDim() {
    return m_vec.getDim();
}

bool Node::empty() const {
    return m_vec.empty();
}

void Node::setVecValue(const float &value, const unsigned int &index) {
    m_vec[index] = value;
}

float Node::getVecValue(const unsigned int &index) {
    return m_vec[index];
}

float Node::norm() const {
    return m_vec.norm();
}

float Node::getDist(const shared_ptr<Node> &node) const {
    return getDist(*node);
}

float Node::getDist(const Node &node) const {
    return m_vec.getDist(node.getVec());
}

float Node::getDistToParent() const {
    if (m_parent == NULL)
        return -1;
    else
        return getDist(m_parent);
}

void Node::setCost(const float &cost) {
    if (cost > 0)
        m_cost = cost;
}

float Node::getCost() const {
    return m_cost;
}

void Node::setParent(const shared_ptr<Node> &parent) {
    m_parent = parent;
}

shared_ptr<Node> Node::getParent() {
    return m_parent;
}

void Node::clearParent() {
    m_parent = NULL;
}

void Node::addChild(const shared_ptr<Node> &child) {
    m_childs.push_back(child);
}

std::vector<shared_ptr<Node>> Node::getChilds() {
    return m_childs;
}

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

Vec<float> Node::getVec() const {
    return m_vec;
}
