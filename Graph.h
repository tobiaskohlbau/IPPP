#ifndef GRAPH_H_
#define GRAPH_H_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

#include "KDTree.h"
#include "Node.h"

using std::shared_ptr;

class Graph
{
public:
    void addNode(shared_ptr<Node> node);
    void removeNode(const int index);
    std::vector<shared_ptr<Node>> getNodes() const;

    shared_ptr<Node> getNearestNode(const Node node);
    std::vector<shared_ptr<Node>> getNearNodes(const shared_ptr<Node> node, const float &distance);

private:
    std::vector<shared_ptr<Node>> m_nodes;
    KDTree<shared_ptr<Node>> m_kdTree;
};

void Graph::addNode(shared_ptr<Node> node) {
    m_nodes.push_back(node);
    m_kdTree.addNode(node->getVec(), node);
}

void Graph::removeNode(const int index) {
    if (index < m_nodes.size()) {
        m_nodes.erase(m_nodes.begin() + index);
    }
}

std::vector<shared_ptr<Node>> Graph::getNodes() const{
    return m_nodes;
}

shared_ptr<Node> Graph::getNearestNode(const Node node) {
    return m_kdTree.searchNearestNeighbor(node.getVec());
}

std::vector<shared_ptr<Node>> Graph::getNearNodes(const shared_ptr<Node> node, const float &distance) {
    return m_kdTree.searchRange(node->getVec(), distance);
}


#endif /* GRAPH_H_ */
